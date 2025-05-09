import openai
import os

from langchain.embeddings import OpenAIEmbeddings
from langchain.vectorstores import FAISS
from langchain.document_loaders import TextLoader
from langchain.text_splitter import CharacterTextSplitter

# ---------- Step 1: Setup API keys and environment ----------
print(os.environ["OPENAI_API_KEY"]) 
# ---------- Step 2: Load and chunk context documents ----------
loader = TextLoader("database.txt")
documents = loader.load()

splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=100)
docs = splitter.split_documents(documents)

# ---------- Step 3: Create FAISS vector store ----------
embeddings = OpenAIEmbeddings()
vectorstore = FAISS.from_documents(docs, embeddings)

retriever = vectorstore.as_retriever()

# ---------- Step 4: Retrieve context for RAG ----------
retrieved_docs = retriever.get_relevant_documents("drone and robot dog collaboration")
retrieved_text = "\n\n".join([doc.page_content for doc in retrieved_docs])

# ---------- Step 5: Build message history with RAG context ----------
message_history = [
    {"role": "system", "content": f"You are a helpful assistant. Use this context if relevant:\n\n{retrieved_text}"},
    {"role": "user", "content": '''Mission Scenario: We have a drone and a robot dog. 
    We also have a mission to pick up a ball from a field of size 50x50 meters. The goal is to locate a ball residing in a 1x1 cell.
    The drone can fly over the field and can determine the 10x10 meters square having the ball. The drone is initially located at point (5,5). Also, it can communicate
    with the robot dog when necessary. The dog robot must scan the area, locate the ball, and bring it back to the starting point located at point (25,25) and has to start scanning from there.
    While scanning the area, the dog can scan up to 2 meters in all directions. So, for example, if it is located at x = 10, it can see the range of x = 8 to x = 12 only) and same for the y direction. 
    Generate a high-level plan for the collaboration between the drone and the robot dog to finish this mission. Make sure that the drone follows a systematic approach to cover the whole field area. 
    Your plan should be of this format:
    Drone Plan:
    Phase 1: ----------------------------
    Phase 2: ---------------------------
    -------
    Dog Plan:
    Phase 1: -----------------------------
    Phase 2: ------------------------------'''},
]

# ---------- Step 6: Query the LLM ----------
client = openai.OpenAI(
    api_key=os.environ["OPENAI_API_KEY"])

response = client.chat.completions.create(
    model='gpt-4o',
    messages=message_history,
    temperature=0.1,
    top_p=0.1
)

'''

   model_name="gpt-4o",
        openai_api_key=OPENAI_API_KEY,
        temperature=0.1
'''
# ---------- Step 7: Write the plan to file ----------
os.makedirs("Plans", exist_ok=True)
with open("Plans/initial_plan.txt", "w") as file:
    file.write(response.choices[0].message.content)

print("Mission Plan is written to Plans/initial_plan.txt")
print(retrieved_text)