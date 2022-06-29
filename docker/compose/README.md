# README

`gen_docker_compose.py`: Script to generate docker compose file for launching
simulation, bridge, and team containers locally.

## Dependencies

Requires python `jinja2`. Install by running the following command:


```sh
sudo apt install python3-jinja2
```

## Usage

Example usage:

```sh
python3 gen_docker_compose.py --config `ros2 pkg prefix mbzirc_seed`/share/mbzirc_seed/config/team.yaml --image mbzirc_seed
```

The script has 2 required args:

* `--config <path_to_team_config_file>`: Team configuration yaml file containing the robots and sensors to spawn into simulation
* `--image <team_image>`: Name of team solution docker image

Other optional args:

* `--world <world_name>`: Name of the world to launch. Defaults to `coast`.
* `--headless <headless>`: `1` or `0`. `1` (True) to launch simulation headless (no GUI). Defaults to `0`.
* `--out <output_file_path>`: Path of output docker compose yaml file. Defaults to `mbzirc_compose.yaml`.
