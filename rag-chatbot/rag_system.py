import os
from dotenv import load_dotenv
from langchain_community.document_loaders import UnstructuredMarkdownLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_community.embeddings import HuggingFaceEmbeddings
from langchain_community.vectorstores import Chroma
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain.chains import create_retrieval_chain
from langchain.chains.combine_documents import create_stuff_documents_chain
from langchain_core.prompts import ChatPromptTemplate
import glob

# Load environment variables
load_dotenv()
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
BOOK_DOCS_PATH = os.getenv("BOOK_DOCS_PATH", "../website/docs")

# --- RAG Components ---
vectorstore = None
rag_chain = None

def get_documents():
    """Loads markdown documents from the specified path."""
    md_files = glob.glob(os.path.join(BOOK_DOCS_PATH, '**', '*.md'), recursive=True)
    md_files.extend(glob.glob(os.path.join(BOOK_DOCS_PATH, '**', '*.mdx'), recursive=True))
    
    documents = []
    for file_path in md_files:
        try:
            loader = UnstructuredMarkdownLoader(file_path)
            documents.extend(loader.load())
        except Exception as e:
            print(f"Error loading {file_path}: {e}")
    return documents

def split_documents(documents):
    """Splits documents into smaller chunks."""
    text_splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=200)
    return text_splitter.split_documents(documents)

def get_embeddings_model():
    """Initializes and returns the HuggingFace embeddings model."""
    # Using a local model for embeddings to avoid external API calls initially
    return HuggingFaceEmbeddings(model_name="all-MiniLM-L6-v2")

def initialize_vectorstore(docs_chunks):
    """Initializes and returns the Chroma vector store."""
    global vectorstore
    embeddings = get_embeddings_model()
    # Using a persistent ChromaDB store for reusability
    vectorstore = Chroma.from_documents(docs_chunks, embeddings, persist_directory="./chroma_db")
    print("Vectorstore initialized with", len(docs_chunks), "chunks.")

def get_llm():
    """Initializes and returns the Google Gemini LLM."""
    if not GEMINI_API_KEY:
        raise ValueError("GEMINI_API_KEY not found in environment variables.")
    return ChatGoogleGenerativeAI(model="gemini-pro", google_api_key=GEMINI_API_KEY)

def setup_rag_chain():
    """Sets up and returns the RAG chain."""
    global rag_chain
    if vectorstore is None:
        raise ValueError("Vectorstore not initialized. Run ingest_documents first.")

    llm = get_llm()
    
    prompt = ChatPromptTemplate.from_messages([
        ("system", "You are an AI tutor for the Physical AI & Humanoid Robotics textbook. Answer the user's question ONLY based on the provided context. If the answer is not in the context, politely state that you don't have enough information to answer from the provided book content."),
        ("user", "Context: {context}\nQuestion: {input}")
    ])
    
    document_chain = create_stuff_documents_chain(llm, prompt)
    retriever = vectorstore.as_retriever()
    rag_chain = create_retrieval_chain(retriever, document_chain)
    print("RAG chain setup complete.")

def ingest_documents_to_rag():
    """Main function to ingest documents and set up the RAG system."""
    print("Starting document ingestion...")
    documents = get_documents()
    if not documents:
        print("No documents found to ingest.")
        return
    docs_chunks = split_documents(documents)
    initialize_vectorstore(docs_chunks)
    setup_rag_chain()
    print("Document ingestion and RAG setup complete.")

def get_rag_response(query):
    """Gets a response from the RAG chain for a given query."""
    if rag_chain is None:
        raise ValueError("RAG chain not set up. Run ingest_documents_to_rag first.")
    response = rag_chain.invoke({"input": query})
    return response['answer']

if __name__ == "__main__":
    # Example usage:
    ingest_documents_to_rag()
    print("\n--- Testing RAG Chain ---")
    question = "what is physical ai?"
    answer = get_rag_response(question)
    print(f"Question: {question}")
    print(f"Answer: {answer}")
