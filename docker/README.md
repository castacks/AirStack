```
# make sure in this directory
# build the image, it is named airlab-autonomy-dev:latest
docker compose build

# run the container, starts a bash session
docker compose run --name airlab_autonomy_dev airlab-autonomy-dev

# spawn more terminals
docker exec -it airlab_autonomy_dev bash
```