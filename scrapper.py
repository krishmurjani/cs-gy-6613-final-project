import os
import shutil
import subprocess
import tempfile
import uuid
import time
import logging
from typing import Dict, Optional
from pymongo import MongoClient, errors
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.chrome.service import Service
from webdriver_manager.chrome import ChromeDriverManager
from bs4 import BeautifulSoup

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def get_mongo_collection(uri: str, db_name: str, collection_name: str):
    # connect to mongodb
    client = MongoClient(uri)
    db = client[db_name]
    return db[collection_name]

class GithubCrawler:
    def __init__(self, mongo_collection, ignore=(".git", ".toml", ".lock", ".png")) -> None:
        self._ignore = ignore
        self.collection = mongo_collection

    def extract(self, link: str, user: Dict) -> None:
        # extract github repo content and save to mongodb
        if self.collection.find_one({"link": link}):
            logger.info(f"repository already exists: {link}")
            return

        logger.info(f"starting github repo scrape: {link}")
        repo_name = link.rstrip("/").split("/")[-1]
        local_temp = tempfile.mkdtemp()

        try:
            subprocess.run(["git", "clone", link], check=True, cwd=local_temp)
            repo_path = os.path.join(local_temp, os.listdir(local_temp)[0])

            tree = {}
            for root, _, files in os.walk(repo_path):
                rel_dir = root.replace(repo_path, "").lstrip("/")
                if any(rel_dir.startswith(pattern) for pattern in self._ignore):
                    continue

                for file in files:
                    if any(file.endswith(pattern) for pattern in self._ignore):
                        continue
                    file_path = os.path.join(rel_dir, file)
                    try:
                        with open(os.path.join(root, file), "r", errors="ignore") as f:
                            tree[file_path] = f.read().strip()
                    except Exception as e:
                        logger.warning(f"failed to read file {file_path}: {e}")

            repo_data = {
                "_id": str(uuid.uuid4()),
                "name": repo_name,
                "link": link,
                "content": tree,
                "platform": "github",
                "author_id": user["id"],
                "author_full_name": user["full_name"],
            }
            self.collection.insert_one(repo_data)
            logger.info(f"repository {repo_name} saved.")
        except subprocess.CalledProcessError as e:
            logger.error(f"failed to clone repo: {e}")
        except errors.PyMongoError as e:
            logger.error(f"failed to save to mongodb: {e}")
        finally:
            shutil.rmtree(local_temp)

        logger.info(f"finished scraping github repo: {link}")

# See notebook for medium crawler
# class MediumCrawler:
#     def __init__(self, mongo_collection):
#         self.collection = mongo_collection
#         self.options = Options()
#         self.options.add_argument("--headless")
#         service = Service(ChromeDriverManager().install())
#         self.driver = webdriver.Chrome(service=service, options=self.options)

#     def scroll_page(self, scroll_pause_time=1):
#         # scroll to load full page
#         last_height = self.driver.execute_script("return document.body.scrollHeight")

#         while True:
#             self.driver.execute_script("window.scrollTo(0, document.body.scrollHeight);")
#             time.sleep(scroll_pause_time)

#             new_height = self.driver.execute_script("return document.body.scrollHeight")
#             if new_height == last_height:
#                 break
#             last_height = new_height

#     def extract(self, link: str, user: Optional[dict] = None):
#         # extract medium article content
#         if self.collection.find_one({"link": link}):
#             logger.info(f"article already exists: {link}")
#             return

#         logger.info(f"starting medium scrape: {link}")
#         self.driver.get(link)
#         self.scroll_page()

#         soup = BeautifulSoup(self.driver.page_source, "html.parser")
#         title = soup.find("h1", class_="pw-post-title")
#         subtitle = soup.find("h2", class_="pw-subtitle-paragraph")

#         data = {
#             "Title": title.get_text(strip=True) if title else None,
#             "Subtitle": subtitle.get_text(strip=True) if subtitle else None,
#             "Content": soup.get_text(strip=True),
#         }

#         doc = {
#             "_id": str(uuid.uuid4()),
#             "link": link,
#             "platform": "medium",
#             "content": data,
#         }
#         if user:
#             doc.update({
#                 "author_id": user["id"],
#                 "author_full_name": user["full_name"],
#             })

#         try:
#             self.collection.insert_one(doc)
#             logger.info(f"article saved: {link}")
#         except errors.PyMongoError as e:
#             logger.error(f"failed to save article: {e}")

#         return data

#     def close(self):
#         # close web driver
#         self.driver.quit()