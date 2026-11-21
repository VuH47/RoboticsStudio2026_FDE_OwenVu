# Portfolio documentation images

Copied into Doxygen HTML via `IMAGE_PATH = docs_assets` (this folder is tracked in git; generated `doc/html` stays ignored).

## Currently included
- `ground_arrival_T0*.png` — Husky standoff arrival evidence
- `ground_verified_T0*.png` — ground visual confirmation evidence

## Recommended additions
| Filename | Caption idea |
|---|---|
| `demo_mission.gif` | Full pin → navigate → confirm mission (30–60 s) |
| `architecture_gazebo.png` | Gazebo `snow_forest` with UAV + UGV |
| `operator_gui.png` | PySide6 / PyVista operator dashboard |
| `aerial_debug.png` | Parrot RGB-D ice detection HUD |
| `detour_rviz.png` | Costmap corridor detour around rock gates |

Rebuild with: `doxygen Doxyfile`
