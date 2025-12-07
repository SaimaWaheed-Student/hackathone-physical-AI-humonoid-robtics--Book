from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import os
from dotenv import load_dotenv
from langchain_community.vectorstores import FAISS
from langchain_google_genai import GoogleGenerativeAIEmbeddings
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain.chains import RetrievalQA

# Load environment variables
load_dotenv()
os.environ["GOOGLE_API_KEY"] = os.getenv("GOOGLE_API_KEY")

app = FastAPI()

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

# Global variables for vector store and QA chain
vector_store = None
qa_chain = None

# Define paths
VECTOR_STORE_PATH = "./vector_store"

class ChatRequest(BaseModel):
    query: str

@app.on_event("startup")
async def startup_event():
    global vector_store, qa_chain
    print("Loading vector store and setting up QA chain...")
    try:
        # Initialize embeddings
        embeddings = GoogleGenerativeAIEmbeddings(model="models/embedding-001")
        # Load the vector store
        vector_store = FAISS.load_local(VECTOR_STORE_PATH, embeddings, allow_dangerous_deserialization=True)
        # Initialize LLM
        llm = ChatGoogleGenerativeAI(model="gemini-pro", temperature=0.7)
        # Create QA chain
        qa_chain = RetrievalQA.from_chain_type(
            llm=llm,
            chain_type="stuff",
            retriever=vector_store.as_retriever(),
            return_source_documents=True
        )
        print("Vector store loaded and QA chain set up.")
    except Exception as e:
        print(f"Error during startup: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to load resources: {e}")

@app.post("/api/chat")
async def chat_endpoint(request: ChatRequest):
    if not qa_chain:
        raise HTTPException(status_code=500, detail="Chatbot not initialized.")
    
    try:
        response = qa_chain({"query": request.query})
        answer = response["result"]
        # Check if the answer is based on the provided documents
        if not response["source_documents"]:
            answer = "This information is not available in the book."
        return {"answer": answer}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/search")
async def search_endpoint(request: ChatRequest):
    if not vector_store:
        raise HTTPException(status_code=500, detail="Vector store not initialized.")
    
    try:
        docs = vector_store.similarity_search(request.query)
        results = [{"content": doc.page_content, "metadata": doc.metadata} for doc in docs]
        return {"results": results}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)