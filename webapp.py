import gradio as gr
from typing import List, Dict
from retriever import OllamaHandler, Retriever, QdrantClient, EmbeddingModel

# init retriever components
qdrant_client = QdrantClient(url="http://qdrant:6333")
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
        # retrieve and generate response
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
            return f"error: {str(e)}"

retriever_with_ollama = RetrieverWithOllama(retriever=retriever, ollama_handler=ollama_handler)

questions = [
    "What is ROS?",
    "Steps to install ROS?",
    "What is the latest version of ROS?",
    "Tell me how can I navigate to a specific pose - include replanning aspects in your answer.",
	"Tell me how can I navigate to a specific pose - include replanning aspects in your answer. Can you provide me with code for this task?"
]

def start_gradio_app(retriever_with_ollama: RetrieverWithOllama):
    # launch gradio app
    def get_answer(query: str) -> str:
        return retriever_with_ollama.retrieve_and_respond(query, top_k=5)

    demo = gr.Interface(
        fn=get_answer,
        inputs=gr.Dropdown(choices=questions, label="Select a question"),
        outputs=gr.Textbox(label="Response", lines=10),
        title="CS-GY-6613 AI Final Project: ROS Query App",
        description="Select a question from the dropdown and then click **Search** to get the answer.",
        article="Created by Shresth Kapoor (sk11677) & Krish Murjani (km6520)"
    ).launch(server_name="0.0.0.0", server_port=7860)

    demo.launch()