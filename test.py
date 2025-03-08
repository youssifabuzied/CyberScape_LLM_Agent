from langchain.vectorstores import Chroma
from langchain.embeddings import OpenAIEmbeddings
from langchain.chains import RetrievalQA
from langchain.document_loaders import TextLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain.llms import OpenAI
from langchain.chat_models import ChatOpenAI
import os
import warnings
warnings.filterwarnings("ignore")


# Load past missions into a vector store
def initialize_vector_store():
    if not os.path.exists("db"):
        os.makedirs("db")
    
    loader = TextLoader("scanning_and_motion_planning.txt")  # Store past missions here
    documents = loader.load()

    text_splitter = RecursiveCharacterTextSplitter(chunk_size=500, chunk_overlap=100)
    texts = text_splitter.split_documents(documents)

    vector_store = Chroma.from_documents(texts, OpenAIEmbedd    ings(model="text-embedding-ada-002"), persist_directory="db")
    vector_store.persist()
    
    return vector_store

# Retrieve relevant past missions
def retrieve_relevant_missions(vector_store, query):
    retriever = vector_store.as_retriever(search_type="similarity", search_kwargs={"k": 3})
    retrieved_docs = retriever.get_relevant_documents(query)
    return "\n".join([doc.page_content for doc in retrieved_docs])

initialize_vector_store()