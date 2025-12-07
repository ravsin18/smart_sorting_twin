import streamlit as st
import paho.mqtt.client as mqtt
import json
import time
import pandas as pd
import queue

# CONFIGURATION
MQTT_BROKER = "127.0.0.1"
MQTT_TOPIC = "robot/telemetry"

st.set_page_config(page_title="Smart Sorting Twin", layout="wide")
st.title("🏭 Digital Twin Monitor")

# --- GLOBAL MAILBOX (Outside Session State) ---
# This allows the background thread to talk to the main thread safely
if 'data_queue' not in st.session_state:
    st.session_state.data_queue = queue.Queue()

# Initialize chart data storage
if 'chart_data' not in st.session_state:
    st.session_state.chart_data = []

# MQTT Callback (Runs in background thread)
def on_message(client, userdata, message):
    try:
        # We parse the message here, but we CANNOT touch st.session_state directly
        payload = json.loads(message.payload.decode())
        
        # We define a queue inside the user data to bridge the gap
        # (This avoids the "missing ScriptRunContext" error)
        userdata.put(payload)
    except Exception as e:
        print(f"Error: {e}")

# Setup MQTT Client
@st.cache_resource
def setup_mqtt():
    # We pass the Queue as 'userdata' so the callback can find it
    q = queue.Queue()
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "Streamlit_Viewer", userdata=q)
    client.on_message = on_message
    try:
        client.connect(MQTT_BROKER, 1883, 60)
        client.subscribe(MQTT_TOPIC)
        client.loop_start()
    except Exception as e:
        st.error(f"MQTT Connection Failed: {e}")
    return client, q

# Start Client and get the shared Queue
client, shared_queue = setup_mqtt()

# LAYOUT
col1, col2 = st.columns(2)
with col1:
    st.subheader("Live Robot Telemetry")
    chart_holder = st.empty()
with col2:
    st.subheader("System Status")
    status_holder = st.empty()

# LIVE UPDATE LOOP
while True:
    # 1. Empty the mailbox (Queue) into our session state
    while not shared_queue.empty():
        new_item = shared_queue.get()
        st.session_state.chart_data.append(new_item)
        
        # Keep list short
        if len(st.session_state.chart_data) > 50:
            st.session_state.chart_data.pop(0)

    # 2. Update the Display
    if len(st.session_state.chart_data) > 0:
        latest = st.session_state.chart_data[-1]
        
        # Update Text
        status_holder.info(f"**Robot ID:** {latest['robot_id']}\n\n**Timestamp:** {latest['timestamp']}")
        
        # Update Chart
        df = pd.DataFrame(st.session_state.chart_data)
        if not df.empty and 'joint_positions' in df.columns:
            joints = pd.DataFrame(df['joint_positions'].to_list())
            joints.columns = [f"J{i+1}" for i in range(joints.shape[1])]
            chart_holder.line_chart(joints[['J1', 'J2']])
            
    time.sleep(0.1)