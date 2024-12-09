from scrapper import GithubCrawler, MediumCrawler
from cleaning import DataPipeline
from retriever import Retriever, OllamaHandler, EmbeddingModel
from webapp import start_gradio_app, RetrieverWithOllama
import uuid
from pymongo import MongoClient
from qdrant_client import QdrantClient
from clearml import Task

def main():
    # Step 0: Initialize ClearML
    task = Task.init(
        project_name="AI Project",
        task_name="Pipeline Execution",
        task_type=Task.TaskTypes.training,
    )
    logger = task.get_logger()

    # Step 1: Initialize MongoDB and Qdrant
    logger.report_text("Initializing MongoDB and Qdrant...")
    print("Initializing MongoDB and Qdrant...")
    mongo_client = MongoClient("mongodb://localhost:27017/")
    github_collection = mongo_client["github_scraper"]["repositories"]
    medium_collection = mongo_client["medium_scraper"]["articles"]

    qdrant_client = QdrantClient(url="http://localhost:6333")

    # Step 2: Scraping
    logger.report_text("Starting scrapers...")
    print("Starting scrapers...")
    github_crawler = GithubCrawler(github_collection)
    medium_crawler = MediumCrawler(medium_collection)

    test_user = {"id": str(uuid.uuid4()), "full_name": "Test User"}
    github_crawler.extract(
        link="https://github.com/ros-controls/ros2_controllers",
        user=test_user
    )

    medium_crawler.extract(
        link="https://medium.com/@tetraengnrng/a-beginners-guide-to-ros2-29721dcf49c8",
        user=test_user
    )
    medium_crawler.close()

    # Step 3: Cleaning with DataPipeline
    logger.report_text("Starting cleaning...")
    print("Starting cleaning...")

    medium_pipeline = DataPipeline(
        mongo_collection=medium_collection,
        qdrant_client=qdrant_client,
        qdrant_collection_name="medium_article_chunks",
    )

    medium_ids = [doc["_id"] for doc in medium_collection.find()]
    logger.report_text(f"Medium article IDs: {medium_ids}")
    for medium_article_id in medium_ids:
        print(f"Processing Medium article ID: {medium_article_id}")
        medium_pipeline.process_medium_article_by_id(medium_article_id)

    github_pipeline = DataPipeline(
        mongo_collection=github_collection,
        qdrant_client=qdrant_client,
        qdrant_collection_name="repository_chunks",
    )

    github_ids = [doc["_id"] for doc in github_collection.find()]
    logger.report_text(f"GitHub repository IDs: {github_ids}")
    for repository_id in github_ids:
        print(f"Processing GitHub repository ID: {repository_id}")
        github_pipeline.process_repository_by_id(repository_id)

    # Step 4: Retrieval and Response
    logger.report_text("Starting retriever...")
    print("Starting retriever...")
    embedding_model = EmbeddingModel()
    retriever = Retriever(
        qdrant_client=qdrant_client,
        github_collection_name="repository_chunks",
        medium_collection_name="medium_article_chunks",
        embedding_model=embedding_model
    )

    ollama_handler = OllamaHandler()
    retriever_with_ollama = RetrieverWithOllama(retriever=retriever, ollama_handler=ollama_handler)

    # Step 5: Launch Gradio App
    logger.report_text("Launching Gradio app...")
    print("Launching Gradio app...")
    start_gradio_app(retriever_with_ollama)


if __name__ == "__main__":
    main()