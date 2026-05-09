# Black Square Frame 3D Corner Detection

Realtime detection of 3D corner coordinates for a black square frame:
- Side length: 10 cm (100 mm)
- Line width: 0.3 cm (3 mm)
- Corner definition: centerline intersections
- Output coordinate system: camera coordinates (Xc, Yc, Zc)

## Install

```bash
pip install -r requirements.txt
```

## Configure

1. Edit `config/camera.yaml` with your calibrated camera matrix and distortion coefficients.
2. Keep `config/detector.yaml` defaults or tune threshold/filter values for your environment.

## Run

```bash
python main.py --camera config/camera.yaml --detector config/detector.yaml --show-binary
```

Controls:
- `q`: quit
- `s`: save current frame

## Notes

- This pipeline solves pose with `cv2.SOLVEPNP_IPPE` for planar 4-point geometry.
- A frame is marked valid only when reprojection error is below configured threshold.
- If no outer-inner quad is found, check lighting/threshold and frame size in `config/detector.yaml`.
