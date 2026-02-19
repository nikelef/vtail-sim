# V-Tail Unpowered Drop — Simulation + 3D Game + Dashboard

This project simulates an unpowered V-tail aircraft dropped from altitude.
The touchdown point is controlled via ruddervators:
- Symmetric deflection => pitch (glide ratio control)
- Differential deflection => yaw (heading/turn control)

## Features
- Speed varies with energy (potential -> kinetic) with simple drag
- Turn-induced sink (bank penalty)
- Explicit ruddervator mixer (left/right surfaces)
- 3D OpenGL chase-view game
- Streamlit + Plotly dashboard for replay plots

## Install
```bash
pip install -r requirements.txt
