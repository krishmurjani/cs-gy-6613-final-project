import re
import numpy as np
from typing import Any, Dict, List
from uuid import uuid4
from pydantic import BaseModel, Field
from pymongo import MongoClient
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct, ScoredPoint
from sentence_transformers import SentenceTransformer
import requests
import json

class DataCategory(str):
    posts = "posts"
    articles = "articles"
    repositories = "repositories"
    queries = "queries"

class BaseDocument(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid4()))

    @classmethod
    def from_mongo(cls, data: Dict):
        # map mongo id to base document id
        data["id"] = data.pop("_id")
        return cls(**data)

class VectorBaseDocument(BaseDocument):
    content: str
    metadata: Dict[str, Any] = Field(default_factory=dict)

class EmbeddingModel:
    def __init__(self):
        # init embedding model
        self.model = SentenceTransformer("sentence-transformers/all-MiniLM-L6-v2")

    def embed(self, texts: List[str]) -> List[List[float]]:
        # generate embeddings
        return self.model.encode(texts, convert_to_tensor=False)

class Retriever:
    def __init__(self, qdrant_client: QdrantClient, github_collection_name: str, medium_collection_name: str, embedding_model: EmbeddingModel):
        self.qdrant_client = qdrant_client
        self.github_collection_name = github_collection_name
        self.medium_collection_name = medium_collection_name
        self.embedding_model = embedding_model

    def retrieve(self, query: str, collection: str, top_k: int = 5) -> List[Dict]:
        # retrieve relevant results from a collection
        query_embedding = self.embedding_model.embed([query])[0]
        search_results: List[ScoredPoint] = self.qdrant_client.search(
            collection_name=collection,
            query_vector=query_embedding,
            limit=top_k,
        )
        return [{"id": res.id, "score": res.score, "payload": res.payload} for res in search_results]

    def retrieve_from_github(self, query: str, top_k: int = 5) -> List[Dict]:
        # retrieve results from github collection
        print(f"retrieving results from github repository collection for query: {query}")
        return self.retrieve(query, collection=self.github_collection_name, top_k=top_k)

    def retrieve_from_medium(self, query: str, top_k: int = 5) -> List[Dict]:
        # retrieve results from medium collection
        print(f"retrieving results from medium article collection for query: {query}")
        return self.retrieve(query, collection=self.medium_collection_name, top_k=top_k)

class OllamaHandler:
    def __init__(self, ollama_url: str = "http://localhost:11434/api/generate"):
        # init ollama api handler
        self.ollama_url = ollama_url

    def send_prompt(self, question: str, results: List[Dict]) -> str:
        # send prompt to ollama api
        formatted_results = "\n".join(
            [f"result {i+1}:\ncontent: {res['payload']['content']}\nscore: {res['score']}\n"
             for i, res in enumerate(results)]
        )
        prompt = f"""
        you are an expert assistant. answer the query based on the provided results:
        query: {question}
        results: {formatted_results}
        provide an insightful and concise answer."""

        response = requests.post(
            url=self.ollama_url,
            json={"model": "llama3.2", "prompt": prompt},
            stream=True,
        )
        responses = []
        for line in response.iter_lines(decode_unicode=True):
            if line.strip():
                data = json.loads(line)
                responses.append(data)
                if data.get("done", False):
                    break
        return "".join(item["response"] for item in responses)