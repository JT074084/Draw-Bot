import streamlit as st
import requests
import plotly.graph_objects as go
import math
import numpy as np

l1 = 80.00
l2 = 150.00
d = 112.25
c = 0.0
e = 0.0

ipAddress = "YOUR_ESP32_IP_ADDRESS"

if "points" not in st.session_state:
    st.session_state.points = []

def interpolatePoints(point1, point2, percentage):
    distance = np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    interpolatedPoints = []

    pointAmount = int(percentage * distance)

    xi = np.linspace(point1[0], point2[0], pointAmount)
    yi = np.linspace(point1[1], point2[1], pointAmount)

    for p in range(pointAmount):
        interpolatedPoints.append((float(xi[p]), float(yi[p])))

    return interpolatedPoints

def pointReachable(x, y):
    # Precheck
    if y < 75.00:
        return False

    leftPointDistance = math.sqrt((x**2) + (y**2))
    rightPointDistance = math.sqrt(((d - x)**2) + (y**2))

    leftPointReachable = abs(l1 - l2) <= leftPointDistance <= (l1 + l2)
    rightPointReachable = abs(l1 - l2) <= rightPointDistance <= (l1 + l2)

    if leftPointReachable and rightPointReachable:
        return True
    else:
        return False
    
@st.cache_data
def getWorkSpaceGrid():
    x_range = np.linspace(-300, 300, 150)
    y_range = np.linspace(0, 400, 150)

    xx, yy = np.meshgrid(x_range, y_range)

    left_dist = np.sqrt(xx**2 + yy**2)
    right_dist = np.sqrt((d - xx)**2 + yy**2)

    left_ok = (abs(l1 - l2) <= left_dist) & (left_dist <= (l1 + l2))
    right_ok = (abs(l1 - l2) <= right_dist) & (right_dist <= (l1 + l2))
    y_ok = yy >= 75.0

    reachable = (left_ok & right_ok & y_ok).astype(int)  # 1 = reachable, 0 = not

    return x_range, y_range, reachable

def drawWorkspace():
    x_range, y_range, reachable = getWorkSpaceGrid()

    fig = go.Figure()

    # Heat Map
    fig.add_trace(go.Heatmap(
        x=x_range,
        y=y_range,
        z=reachable,
        colorscale=[[0, "red"], [1, "green"]],
        opacity=0.4,
        showscale=False
    ))

    # User Points
    x = [p[0] for p in st.session_state.points]
    y = [p[1] for p in st.session_state.points]
    fig.add_trace(go.Scatter(
        x=x,
        y=y,
        mode="lines+markers",
        marker=dict(color="blue", size=8),
        name="Points"
    ))

    fig.update_layout(title="Drawbot Workspace")
    fig.update_xaxes(showgrid=True, gridcolor="lightgrey")
    fig.update_yaxes(showgrid=True, gridcolor="lightgrey")

    st.plotly_chart(fig, use_container_width=True)

def main():
    st.title("Draw Bot UI V1.0")

    with st.form("test_form"):
        xCoord = st.text_input("X Coordinate: ")
        yCoord = st.text_input("Y Coordinate: ")

        #st.write("Coordinates:")
        #st.text(st.session_state.points)

        percentBar = st.slider("Interpolation Percent", min_value=0.0, max_value=1.0, step=.01)

        add = st.form_submit_button("Add")
        if add: 
            if pointReachable(float(xCoord), float(yCoord)):
                newPoint = (float(xCoord), float(yCoord))
                if len(st.session_state.points) >= 1:
                    lastPoint = st.session_state.points[-1]
                    interpolatedPoints = interpolatePoints(lastPoint, newPoint, percentBar)

                    for point in interpolatedPoints:
                        st.session_state.points.append(point)

                    st.session_state.points.append(newPoint)
                    st.rerun()
                else:
                    st.session_state.points.append(newPoint)
                    st.rerun()
            else:
                st.error("Unreachable Point", icon="❌")
                st.toast("Unreachable Point", icon="❌")

    send = st.button("Send", type="primary")
    if send:
        st.toast("Points Sent", icon="✅")
        message = requests.post(f"http://{ipAddress}/points", json=st.session_state.points)
        print(f"Message Type: {message.status_code}")
        print(f"Message: {message.text}")

    clear = st.button("Clear Points")
    if clear:
        st.session_state.points.clear()
        st.rerun()
        st.toast("Points Cleared")

    drawWorkspace()

if __name__ == "__main__":
    main()
