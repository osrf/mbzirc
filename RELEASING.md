# MBZIRC Releasing instructions

The MBZIRC project releases the software ready to use inside a docker image.
The docker images are available from https://hub.docker.com/u/osrf/mbzirc

This document covers the internal team instructions for releasing a new version
of the docker image

## How to push a new docker image to the dockerhub automatically

The process is fully automated using the GitHub actions environment to build
the docker image and upload it to the dockerhub.

### Triggering a new release

The GitHub action for releasing is configured to react to the new push of git
tags with the following name: `dockerhub_release_*` where `*` is any string.
The string is not used during the release process. It mainly serves as
something meaningful for the person doing the release, could be a name, a date,
or other.

The source code of the new image would the one corresponding to the tag. An
example using `_alpha0`: in a local checkout just run

``` bash
cd mbzirc
git tag dockerhub_release_alpha0
git push --tags
```

In this moment the [Actions tab in the MBZIRC](https://github.com/osrf/mbzirc/actions)
repository should display a new job named "Release to dockerhub", it will take about
15-20 minutes to be completed.

After it finishes successfully two new images will be uploaded to [dockerhub](
https://hub.docker.com/r/osrf/mbzirc/tags)
 * `mbzirc_sim_latest`: this docker image is being updated to the latest release
 * `mbzirc_sim_${timestamp}`: this docker image should have the date of the release

The timestamp includes hour/minute so more than one release can be done in the same day.

### Understanding the GitHub action

The Github Action for releasing mainly does the following:

 1. Setup docker and sources
 2. Use the `build.sh` script to create the docker image from the commit
    corresponding to the initial tag that triggered the action.
 3. Create the docker tags: one is called `mbzric_sim_latest` and the other
    `mbzirc_sim_${timestamp}`. Timestamp includes date and hour-minute.
 4. Login into the dockerhub (requires setup of credentials in this repo)
 5. Push tags to the dockerhub

### dockerhub credentials

The Github Action uses credentials to access to the dockerhub osrf/mbzirc.
The credentials are stored in [Github repository for secrets](
https://github.com/osrf/mbzirc/settings/secrets/actions).

### Troubleshooting

If the Github action fails, the best way to debug it is to [examine its logs](https://docs.github.com/en/actions/monitoring-and-troubleshooting-workflows/using-workflow-run-logs).

1. `rosdep` failed while building the image
Sometimes the network connection with github repositories is broken when
running rosdep. Best way to workaround the problem is usually retry the
build from the Github action UI in "Re-run all jobs" button inside a run.

2. Credentials problems or docker push problems
If for some reason the login step inside dockerhub fails, first step would be
to check that credentials in place inside github repository. Opening
https://github.com/osrf/mbzirc/settings/secrets/actions should display all the
variables are named `secrets.*` in the github action, mainly `DOCKERHUB_USERNAME`
and `DOCKERHUB_PASSWORD`.

The credentials can be tested locally using:
```bash
docker login -u osrf # change `osrf` to your username and enter password
```

## Fallback plan for releasing: manually submit the image to dockerhub

This procedure is not recommended unless the github action is failing to build and release.
Basically it does the same that is run inside the GitHub action but in the user local system:


1. Run the instructions for building a docker image detailed in the README.md
   file of the osrf/mbzirc repository.
2. Tag the images for the dockerhub repository:
   ```bash
   docker tag mbzirc_sim:latest osrf/mbzirc:mbzirc_sim_latest
   docker tag mbzirc_sim:latest osrf/mbzirc:mbzirc_sim_$(date +'%F_%H%M')
   ```
3. Grab crendentials from the OR password manager. Login in dockerhub:
   ```bash
   docker login -u osrf # change `osrf` to your username and enter password
   ```
4. Push the new images:
   ```bash
   docker push osrf/mbzirc:mbzirc_sim_latest
   docker push osrf/mbzirc:mbzirc_sim_$(date +'%F_%H%M')
  ```
