# Robot Control Server

A comprehensive Flask-based REST API server for controlling soft robots via MuJoCo simulation or real hardware connections.

## Features

- 🤖 **Dual Mode Operation**: Switch between MuJoCo simulation and real robot control
- 📡 **REST API**: Complete RESTful API with proper HTTP methods and status codes  
- 📚 **Swagger Documentation**: Auto-generated API documentation at `/docs/`
- 🛡️ **Input Validation**: Comprehensive data validation and error handling
- 🔌 **USB Communication**: Real robot control via USB (with dummy implementation)
- 🎮 **Motor Control**: Precise 3-motor force control with safety limits
- 🚨 **Emergency Stop**: Immediate safety shutdown functionality
- 🔧 **Modern Tooling**: Uses `uv` for fast dependency management

## Quick Start

### Prerequisites

- Python 3.8 or higher
- [uv](https://github.com/astral-sh/uv) package manager

### Installation

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd robot
   ```

2. **Install uv (if not already installed):**
   ```bash
   # On macOS and Linux
   curl -LsSf https://astral.sh/uv/install.sh | sh
   
   # On Windows
   powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
   
   # Or with pip
   pip install uv
   ```

3. **Install dependencies:**
   ```bash
   uv sync
   ```

4. **Start the server:**
   ```bash
   uv run python src/main.py --debug
   ```

The server will start at `http://localhost:5000` with Swagger documentation available at `http://localhost:5000/docs/`.

### Development Setup

Install development dependencies:
```bash
uv sync --extra dev
```

### Optional Dependencies

Install additional features as needed:
```bash
# ROS2 integration
uv sync --extra ros2

# Audio processing for voice control  
uv sync --extra audio

# Computer vision
uv sync --extra vision

# Install everything
uv sync --extra all
```

## API Documentation

### Base URL
```
http://localhost:5000/api/v1
```

### Interactive Documentation
Visit `http://localhost:5000/docs/` for complete interactive Swagger documentation.

### Key Endpoints

#### Robot Status
- **GET** `/api/v1/status/` - Get current robot status

#### Mode Control  
- **GET** `/api/v1/mode/` - Get current mode
- **POST** `/api/v1/mode/` - Set robot mode (simulation/real)

#### Motor Control
- **POST** `/api/v1/motor/command` - Send motor command
- **POST** `/api/v1/motor/emergency_stop` - Emergency stop

#### Generic Commands
- **POST** `/api/v1/command/` - Send generic robot command

### Example API Usage

#### Get Robot Status
```bash
curl -X GET http://localhost:5000/api/v1/status/
```

#### Send Motor Command
```bash
curl -X POST http://localhost:5000/api/v1/motor/command \
  -H "Content-Type: application/json" \
  -d '{
    "motor_1_force": 1.5,
    "motor_2_force": -2.0, 
    "motor_3_force": 0.5,
    "duration": 2.0
  }'
```

#### Switch to Real Robot Mode
```bash
curl -X POST http://localhost:5000/api/v1/mode/ \
  -H "Content-Type: application/json" \
  -d '{"mode": "real"}'
```

#### Emergency Stop
```bash
curl -X POST http://localhost:5000/api/v1/motor/emergency_stop
```

## Development

### Code Quality

Format code:
```bash
uv run black src/ tests/
```

Lint code:
```bash  
uv run pylint src/
```

Type checking:
```bash
uv run mypy src/
```

### Testing

Run tests:
```bash
uv run pytest
```

With coverage:
```bash
uv run pytest --cov=src --cov-report=html
```

### VS Code Setup

The project includes VS Code configuration files:
- `.vscode/settings.json` - Editor settings and Python configuration
- `.vscode/launch.json` - Debug configurations  
- `.vscode/tasks.json` - Build and development tasks

## Project Structure

```
robot/
├── src/                        # Main source code
│   ├── __init__.py            # Package initialization
│   ├── main.py                # Server entry point
│   ├── flask_server.py        # Flask server with Swagger docs
│   ├── schemas.py             # Data schemas and validation
│   ├── mujoco_controller.py   # MuJoCo simulation controller
│   └── usb_controller.py      # USB robot controller
├── tests/                     # Test files
├── .vscode/                   # VS Code configuration
├── pyproject.toml            # Project configuration and dependencies
├── uv.lock                   # Locked dependency versions
└── README.md                 # This file
```

## Configuration

### Environment Variables

- `FLASK_ENV` - Flask environment (development/production)
- `UV_PROJECT_ENVIRONMENT` - uv virtual environment path

### Robot Configuration

The server automatically detects MuJoCo model files in:
1. ROS2 package share directory: `sim/resource/physics_model/spiral_3D.xml`
2. Fallback: `models/default_robot.xml`

## Safety Features

- **Force Limits**: Motor forces are limited to [-10.0, 10.0] range
- **Input Validation**: All API inputs are validated before processing
- **Emergency Stop**: Immediate motor shutdown capability
- **Error Handling**: Comprehensive error responses with proper HTTP status codes

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests and quality checks
5. Submit a pull request

## License

This project is licensed under the MIT License.