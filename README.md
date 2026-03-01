# SpineRebel: Continuous Spine Monitoring

[![IEEE Sensors Letters](https://img.shields.io/badge/IEEE-Sensors%20Letters-blue)](https://ieee-sensors.org/sensors-letters/)

This repository contains code and data associated with the paper:

> **[Paper Title]**  
> [Author(s)]  
> *IEEE Sensors Letters*, [Year]  
> DOI: [DOI link]

## Abstract

[Paper abstract goes here.]

## Repository Structure

```
spinerebel/
├── data/               # Sample datasets and experiment recordings
├── notebooks/          # Jupyter notebooks for analysis and visualization
├── src/                # Source code for signal processing and monitoring
│   ├── acquisition/    # Sensor data acquisition
│   ├── processing/     # Signal processing algorithms
│   └── visualization/  # Visualization utilities
├── requirements.txt    # Python dependencies
└── README.md
```

## Requirements

- Python 3.8+
- See `requirements.txt` for full list of dependencies

## Installation

```bash
git clone https://github.com/gmiekisiak/spinerebel.git
cd spinerebel
pip install -r requirements.txt
```

## Usage

1. **Data acquisition**: See `src/acquisition/` for scripts to interface with the sensor hardware.
2. **Signal processing**: Run the processing pipeline in `src/processing/`.
3. **Visualization**: Use the notebooks in `notebooks/` to reproduce the figures from the paper.

## Citation

If you use this code or data in your research, please cite:

```bibtex
@article{[citekey],
  author  = {[Author(s)]},
  title   = {[Paper Title]},
  journal = {IEEE Sensors Letters},
  year    = {[Year]},
  volume  = {},
  number  = {},
  pages   = {},
  doi     = {}
}
```

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
