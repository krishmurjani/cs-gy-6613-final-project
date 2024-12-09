import re
import numpy as np
import uuid
from enum import Enum
from uuid import uuid4
from typing import Any, Dict, List
from pydantic import BaseModel, Field
from pymongo import MongoClient
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
from sentence_transformers import SentenceTransformer

class DataCategory(str, Enum):
    POSTS = "posts"
    ARTICLES = "articles"
    REPOSITORIES = "repositories"
    QUERIES = "queries"


class BaseDocument(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid4()))

    @classmethod
    def from_mongo(cls, data: dict):
        """Convert MongoDB document to a BaseDocument."""
        data["id"] = data.pop("_id")
        return cls(**data)


class VectorBaseDocument(BaseDocument):
    content: str
    metadata: Dict[str, Any] = Field(default_factory=dict)


class CleaningHandler:
    @staticmethod
    def _clean_text(text: str) -> str:
        """Clean text by removing unwanted characters and extra spaces."""
        if not isinstance(text, str):  # Ensure the input is a string
            text = str(text) if text is not None else ""
        text = re.sub(r"[^\w\s.,!?]", " ", text)
        return re.sub(r"\s+", " ", text).strip()

    def clean_repository(self, data: Dict) -> Dict:
        return {
            "_id": data["_id"],
            "content": self._clean_text(" ".join(data.get("content", {}).values())),
            "name": data.get("name"),
            "link": data.get("link"),
            "platform": data.get("platform"),
        }

    def clean_post(self, data: Dict) -> Dict:
        return {
            "id": data["id"],
            "content": self._clean_text(" ".join(data.get("content", {}).values())),
            "platform": data.get("platform"),
            "image": data.get("image"),
        }

    def clean_article(self, data: Dict) -> Dict:
        """Clean a Medium article's content."""
        content_data = data.get("content", {})
        return {
            "id": data.get("_id", str(uuid.uuid4())),  # Ensure `id` is available
            "title": self._clean_text(content_data.get("Title", "")),
            "subtitle": self._clean_text(content_data.get("Subtitle", "")),
            "content": self._clean_text(content_data.get("Content", "")),
            "link": data.get("link", ""),
            "platform": data.get("platform", "medium"),
        }


def chunk_text(text: str, chunk_size: int = 500, chunk_overlap: int = 50) -> List[str]:
    """
    Split text into chunks of specified size with overlap.
    """
    sentences = re.split(r"(?<!\w\.\w.)(?<![A-Z][a-z]\.)(?<=\.|\?|\!)\s", text)
    chunks, current_chunk = [], ""

    for sentence in sentences:
        if len(current_chunk) + len(sentence) <= chunk_size:
            current_chunk += sentence + " "
        else:
            chunks.append(current_chunk.strip())
            current_chunk = sentence + " "

    if current_chunk:
        chunks.append(current_chunk.strip())

    return chunks


class ChunkingHandler:
    def chunk(self, cleaned_content: str) -> List[Dict[str, str]]:
        """Split cleaned content into chunks and return as dictionaries."""
        chunks = chunk_text(cleaned_content)
        return [{"content": chunk} for chunk in chunks]


class EmbeddingModel:
    def __init__(self):
        self.model = SentenceTransformer("sentence-transformers/all-MiniLM-L6-v2")

    def embed(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a list of texts."""
        return self.model.encode(texts, convert_to_tensor=False)


class EmbeddingHandler:
    def __init__(self, embedding_model: EmbeddingModel):
        self.embedding_model = embedding_model

    def embed_chunks(self, chunks: List[Dict[str, str]]) -> List[Dict[str, Any]]:
        """Embed text chunks and return them with their embeddings."""
        embedded_chunks = []
        for chunk in chunks:
            embedding = self.embedding_model.embed([chunk["content"]])[0]
            embedded_chunks.append({
                "content": chunk["content"],
                "embedding": embedding,
                "metadata": chunk.get("metadata", {}),
            })
        return embedded_chunks


def convert_numpy_to_list(data: Any) -> Any:
    """Recursively convert NumPy arrays to Python lists."""
    if isinstance(data, np.ndarray):
        return data.tolist()
    elif isinstance(data, dict):
        return {k: convert_numpy_to_list(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [convert_numpy_to_list(v) for v in data]
    return data


class DataPipeline:
    def __init__(self, mongo_collection, qdrant_client: QdrantClient, qdrant_collection_name: str):
        self.mongo_collection = mongo_collection
        self.qdrant_client = qdrant_client
        self.qdrant_collection = qdrant_collection_name
        self.cleaning_handler = CleaningHandler()
        self.chunking_handler = ChunkingHandler()
        self.embedding_handler = EmbeddingHandler(EmbeddingModel())

        self._ensure_qdrant_collection()

    def _ensure_qdrant_collection(self):
        """Ensure the Qdrant collection exists, create it if not."""
        collections = self.qdrant_client.get_collections()
        if self.qdrant_collection not in [c.name for c in collections.collections]:
            self.qdrant_client.create_collection(
                collection_name=self.qdrant_collection,
                vectors_config=VectorParams(size=384, distance=Distance.COSINE),
            )

    def process_medium_article_by_id(self, article_id: str):
        """Fetch Medium article by ID, process, and store it in Qdrant."""
        raw_data = self.mongo_collection.find_one({"_id": article_id})
        if not raw_data:
            print(f"No article found with id: {article_id}")
            return

        cleaned_data = self.cleaning_handler.clean_article(raw_data)
        chunks = self.chunking_handler.chunk(cleaned_data["content"])
        embedded_chunks = self.embedding_handler.embed_chunks(chunks)

        points = [
            PointStruct(
                id=str(uuid4()),
                vector=chunk["embedding"],
                payload={"article_id": article_id, "content": chunk["content"]},
            )
            for chunk in embedded_chunks
        ]
        self.qdrant_client.upsert(collection_name=self.qdrant_collection, points=points)
        print(f"Medium article {article_id} processed and stored in Qdrant successfully.")

    def process_repository_by_id(self, repo_id: str):
        """Fetch raw data from MongoDB, process it, and store in Qdrant."""
        raw_data = self.mongo_collection.find_one({"_id": repo_id})
        if not raw_data:
            print(f"No repository found with id: {repo_id}")
            return

        cleaned_data = self.cleaning_handler.clean_repository(raw_data)
        chunks = self.chunking_handler.chunk(cleaned_data["content"])
        embedded_chunks = self.embedding_handler.embed_chunks(chunks)

        points = [
            PointStruct(
                id=str(uuid4()),
                vector=chunk["embedding"],
                payload={"repository_id": repo_id, "content": chunk["content"]},
            )
            for chunk in embedded_chunks
        ]
        self.qdrant_client.upsert(collection_name=self.qdrant_collection, points=points)
        print(f"Repository {repo_id} processed and stored in Qdrant successfully.")