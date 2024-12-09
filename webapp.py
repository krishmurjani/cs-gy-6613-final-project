import gradio as gr
from typing import List, Dict
from retriever import OllamaHandler, Retriever, QdrantClient, EmbeddingModel

qdrant_client = QdrantClient(url="http://localhost:6333")
retriever = Retriever(
    qdrant_client=qdrant_client,
    github_collection_name="repository_chunks",
    medium_collection_name="medium_article_chunks",
    embedding_model=EmbeddingModel()
)
ollama_handler = OllamaHandler()

class RetrieverWithOllama:
    def __init__(self, retriever: Retriever, ollama_handler: OllamaHandler):
        self.retriever = retriever
        self.ollama_handler = ollama_handler

    def retrieve_and_respond(self, query: str, top_k: int = 5) -> str:
        try:
            github_results = self.retriever.retrieve(
                query, top_k=top_k, collection=self.retriever.github_collection_name
            )

            medium_results = self.retriever.retrieve(
                query, top_k=top_k, collection=self.retriever.medium_collection_name
            )

            combined_results = github_results + medium_results

            return self.ollama_handler.send_prompt(query, combined_results)
        except Exception as e:
            return f"Error: {str(e)}"

retriever_with_ollama = RetrieverWithOllama(retriever=retriever, ollama_handler=ollama_handler)

questions = [
    "What is ROS?",
    "Steps to install ROS?",
    "What is the latest version of ROS?",
    "How can I navigate to a specific pose?",
    "What are the benefits of cloud computing?"
]


def start_gradio_app(retriever_with_ollama: RetrieverWithOllama):
    """
    Start the Gradio application using the provided retriever with Ollama.

    Args:
        retriever_with_ollama (RetrieverWithOllama): A retriever that integrates
                                                     retrieval and response generation.
    """
    def get_answer(query: str) -> str:
        return retriever_with_ollama.retrieve_and_respond(query, top_k=5)

    demo = gr.Interface(
        fn=get_answer,
        inputs=gr.Dropdown(choices=questions, label="Select a question"),
        outputs=gr.Textbox(label="Response", lines=10),
        title="ROS Query Application",
        description="Select a question from the dropdown and then click **Search** to get the answer.",
    )

    demo.launch()