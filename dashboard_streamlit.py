import pandas as pd
import numpy as np
import streamlit as st
import plotly.graph_objects as go

from core_sim import SimParams, reset_state, step_sim

st.set_page_config(page_title="V-Tail Glide Dashboard", layout="wide")

st.title("V-Tail Unpowered Drop — Dashboard (Energy + Turn Sink + Ruddervator Mixer)")

with st.sidebar:
    st.header("Scenario")
    start_alt = st.slider("Start altitude [m]", 200, 2000, 900, 50)
    wind_x = st.slider("Wind X [m/s] (east+)", -10.0, 10.0, 2.2, 0.1)
    wind_y = st.slider("Wind Y [m/s] (north+)", -10.0, 10.0, -1.0, 0.1)
    gust = st.slider("Gust amplitude [m/s]", 0.0, 2.0, 0.35, 0.05)

    st.header("Controls (scripted)")
    st.caption("This dashboard uses a simple scripted control profile (you can edit it).")
    pitch_bias = st.slider("Pitch bias", -1.0, 1.0, 0.10, 0.01)
    yaw_bias = st.slider("Yaw bias", -1.0, 1.0, 0.00, 0.01)

    st.header("Physics knobs")
    drag_k = st.slider("Drag k", 0.001, 0.05, 0.012, 0.001)
    bank_sink_factor = st.slider("Turn sink factor", 0.0, 1.0, 0.30, 0.02)
    yaw_rate = st.slider("Yaw rate [deg/s]", 10.0, 120.0, 55.0, 1.0)

    run = st.button("Run simulation")

def run_sim():
    p = SimParams()
    p.start_alt_m = float(start_alt)
    p.wind_x_mps = float(wind_x)
    p.wind_y_mps = float(wind_y)
    p.gust_mps = float(gust)
    p.drag_k = float(drag_k)
    p.bank_sink_factor = float(bank_sink_factor)
    p.yaw_rate_deg_s = float(yaw_rate)

    s = reset_state(p)

    dt = 1/60
    # Simple scripted “pilot”: bias + small time-based adjustments
    # You can replace with your own control law.
    while s.alive and s.t_s < 600:
        # Example: slight pitch-up initially, then reduce near end; mild yaw oscillation
        pitch_in = 0.0
        yaw_in = 0.0

        # Convert bias into incremental command tendency
        pitch_in += pitch_bias
        yaw_in += yaw_bias + 0.25 * np.sin(0.12 * s.t_s)

        # Ease pitch-down near ground to control overshoot
        if s.alt_m < 250:
            pitch_in -= 0.35

        # Limit
        pitch_in = float(np.clip(pitch_in, -1, 1))
        yaw_in = float(np.clip(yaw_in, -1, 1))

        step_sim(p, s, dt, pitch_in, yaw_in)

    df = pd.DataFrame(s.hist)
    impact = None
    if df.shape[0] > 0:
        impact = {
            "t_s": float(df["t_s"].iloc[-1]),
            "x_m": float(df["x_m"].iloc[-1]),
            "y_m": float(df["y_m"].iloc[-1]),
            "miss_m": float(df["miss_m"].iloc[-1]),
            "min_miss_m": float(df["miss_m"].min()),
        }
    return p, df, impact

if run:
    p, df, impact = run_sim()
    if df.empty:
        st.warning("No history produced.")
        st.stop()

    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Flight time [s]", f"{df['t_s'].iloc[-1]:.1f}")
    c2.metric("Impact miss [m]", f"{df['miss_m'].iloc[-1]:.1f}")
    c3.metric("Min miss during flight [m]", f"{df['miss_m'].min():.1f}")
    c4.metric("Impact speed [m/s]", f"{df['v_air_mps'].iloc[-1]:.1f}")

    st.subheader("Top-down track")
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=df["x_m"], y=df["y_m"],
        mode="lines", name="Track"
    ))
    # Target ring
    th = np.linspace(0, 2*np.pi, 200)
    fig.add_trace(go.Scatter(
        x=p.target_x_m + p.target_radius_m*np.cos(th),
        y=p.target_y_m + p.target_radius_m*np.sin(th),
        mode="lines", name="Target radius"
    ))
    fig.add_trace(go.Scatter(
        x=[p.target_x_m], y=[p.target_y_m],
        mode="markers", name="Target"
    ))
    fig.update_layout(
        xaxis_title="x [m]",
        yaxis_title="y [m]",
        height=520,
        yaxis_scaleanchor="x",
        margin=dict(l=20, r=20, t=30, b=20),
    )
    st.plotly_chart(fig, use_container_width=True)

    st.subheader("Time histories")
    colA, colB = st.columns(2)

    with colA:
        fig1 = go.Figure()
        fig1.add_trace(go.Scatter(x=df["t_s"], y=df["alt_m"], mode="lines", name="Altitude [m]"))
        fig1.add_trace(go.Scatter(x=df["t_s"], y=df["v_air_mps"], mode="lines", name="Airspeed [m/s]"))
        fig1.update_layout(height=420, margin=dict(l=20, r=20, t=30, b=20))
        st.plotly_chart(fig1, use_container_width=True)

    with colB:
        fig2 = go.Figure()
        fig2.add_trace(go.Scatter(x=df["t_s"], y=df["bank_deg"], mode="lines", name="Bank [deg]"))
        fig2.add_trace(go.Scatter(x=df["t_s"], y=df["gr"], mode="lines", name="Glide ratio (effective)"))
        fig2.update_layout(height=420, margin=dict(l=20, r=20, t=30, b=20))
        st.plotly_chart(fig2, use_container_width=True)

    st.subheader("Ruddervators (explicit left/right surfaces)")
    fig3 = go.Figure()
    fig3.add_trace(go.Scatter(x=df["t_s"], y=df["left_rv"], mode="lines", name="Left ruddervator"))
    fig3.add_trace(go.Scatter(x=df["t_s"], y=df["right_rv"], mode="lines", name="Right ruddervator"))
    fig3.add_trace(go.Scatter(x=df["t_s"], y=df["pitch_eff"], mode="lines", name="Pitch effective"))
    fig3.add_trace(go.Scatter(x=df["t_s"], y=df["yaw_eff"], mode="lines", name="Yaw effective"))
    fig3.update_layout(height=420, margin=dict(l=20, r=20, t=30, b=20))
    st.plotly_chart(fig3, use_container_width=True)

    st.subheader("Raw data")
    st.dataframe(df, use_container_width=True)
else:
    st.info("Set parameters in the sidebar and click **Run simulation**.")
