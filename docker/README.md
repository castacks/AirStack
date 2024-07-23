```
# make sure in this directory
# build the image, it is named airlab-autonomy-dev:latest
docker compose --profile build build

# start docker compose service
docker compose up -d 

# enter a bash session
docker compose exec airstack_dev bash

# stop service
docker compose stop 
```