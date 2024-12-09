from scrapper import GithubCrawler
from cleaning import DataPipeline
from retriever import Retriever, OllamaHandler, EmbeddingModel
from webapp import start_gradio_app, RetrieverWithOllama
import uuid
from pymongo import MongoClient
from qdrant_client import QdrantClient
# from clearml import Task

def main():

    # Works locally but requires authentication so commented for smooth running please see screenshots in readme
    # init clearml task
    # task = Task.init(
    #     project_name="ai project",
    #     task_name="pipeline execution",
    #     task_type=Task.TaskTypes.training,
    # )
    # logger = task.get_logger()

    # init mongodb and qdrant
    # logger.report_text("initializing mongodb and qdrant...")
    print("initializing mongodb and qdrant...")
    mongo_client = MongoClient("mongodb://mongodb:27017/")
    github_collection = mongo_client["github_scraper"]["repositories"]
    medium_collection = mongo_client["medium_scraper"]["articles"]
    qdrant_client = QdrantClient(url="http://qdrant:6333")

    # start scrapers
    # logger.report_text("starting scrapers...")
    print("starting scrapers...")
    github_crawler = GithubCrawler(github_collection)
    # medium_crawler = MediumCrawler(medium_collection)

    test_user = {"id": str(uuid.uuid4()), "full_name": "test user"}

    github_crawler.extract(
        link="https://github.com/ros-controls/ros2_controllers",
        user=test_user
    )

    # See notebook for medium results
    # medium_crawler.extract(
    #     link="https://medium.com/@tetraengnrng/a-beginners-guide-to-ros2-29721dcf49c8",
    #     user=test_user
    # )
    # medium_crawler.close()

    # clean scraped data
    # logger.report_text("starting cleaning...")
    print("starting cleaning...")

    medium_pipeline = DataPipeline(
        mongo_collection=medium_collection,
        qdrant_client=qdrant_client,
        qdrant_collection_name="medium_article_chunks",
    )

    medium_ids = [doc["_id"] for doc in medium_collection.find()]
    # logger.report_text(f"medium article ids: {medium_ids}")
    for medium_article_id in medium_ids:
        print(f"processing medium article id: {medium_article_id}")
        medium_pipeline.process_medium_article_by_id(medium_article_id)

    github_pipeline = DataPipeline(
        mongo_collection=github_collection,
        qdrant_client=qdrant_client,
        qdrant_collection_name="repository_chunks",
    )

    github_ids = [doc["_id"] for doc in github_collection.find()]
    # logger.report_text(f"github repository ids: {github_ids}")
    for repository_id in github_ids:
        print(f"processing github repository id: {repository_id}")
        github_pipeline.process_repository_by_id(repository_id)

    # start retriever
    # logger.report_text("starting retriever...")
    print("starting retriever...")
    embedding_model = EmbeddingModel()
    retriever = Retriever(
        qdrant_client=qdrant_client,
        github_collection_name="repository_chunks",
        medium_collection_name="medium_article_chunks",
        embedding_model=embedding_model
    )

    ollama_handler = OllamaHandler()
    retriever_with_ollama = RetrieverWithOllama(retriever=retriever, ollama_handler=ollama_handler)

    # launch web app
    # logger.report_text("launching gradio app...")
    print("launching gradio app...")
    start_gradio_app(retriever_with_ollama)


if __name__ == "__main__":
    main()
