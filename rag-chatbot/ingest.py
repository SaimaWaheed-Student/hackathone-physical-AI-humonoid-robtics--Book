import os
from langchain_community.document_loaders import DirectoryLoader, UnstructuredMarkdownLoader
from langchain_text_splitters import CharacterTextSplitter
from langchain_community.vectorstores import FAISS
from langchain_google_genai import GoogleGenerativeAIEmbeddings
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Define paths
DOCS_PATH = "../website/docs"
VECTOR_STORE_PATH = "./vector_store"

def ingest_data():
    """
    Ingests data from the documentation folder, creates embeddings, and stores them in a FAISS vector store.
    """
    print("Starting data ingestion...")

    # Load documents from the documentation folder
    print(f"Loading documents from: {DOCS_PATH}")
    loader = DirectoryLoader(DOCS_PATH, glob="**/*.md", loader_cls=UnstructuredMarkdownLoader, show_progress=True)
    documents = loader.load()
    print(f"Loaded {len(documents)} documents.")

    # Split documents into smaller chunks
    print("Splitting documents into chunks...")
    text_splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=100)
    texts = text_splitter.split_documents(documents)
    print(f"Split documents into {len(texts)} chunks.")

    # Create embeddings
    print("Creating embeddings...")
    embeddings = GoogleGenerativeAIEmbeddings(model="models/embedding-001")

    # Create and save the vector store
    print("Creating and saving the vector store...")
    vector_store = FAISS.from_documents(texts, embeddings)
    vector_store.save_local(VECTOR_STORE_PATH)
    print("Vector store created and saved successfully.")
    print("Data ingestion complete.")

if __name__ == "__main__":
    ingest_data()
