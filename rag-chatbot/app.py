from flask import Flask, request, jsonify
from flask_cors import CORS
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

app = Flask(__name__)
CORS(app) # Enable CORS for all routes

from rag_system import ingest_documents_to_rag, get_rag_response

@app.route("/chat", methods=["POST"])
def chat():
    user_message = request.json.get("message")
    if not user_message:
        return jsonify({"error": "Message is required"}), 400

    if get_rag_response is None:
        return jsonify({"error": "Chatbot is not initialized. Please ensure ingestion is complete."}), 503

    try:
        response_text = get_rag_response(user_message)
        return jsonify({"response": response_text})
    except ValueError as ve:
        print(f"RAG system error: {ve}")
        return jsonify({"error": str(ve)}), 500
    except Exception as e:
        print(f"Error during chat: {e}")
        return jsonify({"error": "Internal server error during chat."}), 500

@app.route("/ingest", methods=["POST"])
def ingest():
    try:
        ingest_documents_to_rag()
        return jsonify({"message": "Ingestion completed successfully!"}), 200
    except ValueError as ve:
        print(f"Ingestion error: {ve}")
        return jsonify({"error": str(ve)}), 500
    except Exception as e:
        print(f"Error during ingestion: {e}")
        return jsonify({"error": "Internal server error during ingestion."}), 500

if __name__ == "__main__":
    # Attempt to ingest documents and set up RAG on startup
    # This will load existing chroma_db or create a new one
    try:
        ingest_documents_to_rag()
    except Exception as e:
        print(f"WARNING: Initial RAG setup failed: {e}. Chat will not be fully functional until /ingest is called successfully.")
    
    app.run(host="0.0.0.0", port=5000, debug=True)