FROM ghcr.io/ad-sdl/madsci:latest

LABEL org.opencontainers.image.source=https://github.com/AD-SDL/openarm_module
LABEL org.opencontainers.image.description="Drivers and REST API's for the openarm robots"
LABEL org.opencontainers.image.licenses=MIT

#########################################
# Module specific logic goes below here #
#########################################

RUN mkdir -p openarm_module

COPY ./src openarm_module/src
COPY ./README.md openarm_module/README.md
COPY ./pyproject.toml openarm_module/pyproject.toml

RUN --mount=type=cache,target=/root/.cache \
    pip install -e ./openarm_module

CMD ["python", "openarm_module/scripts/openarm_rest_node.py"]

#########################################