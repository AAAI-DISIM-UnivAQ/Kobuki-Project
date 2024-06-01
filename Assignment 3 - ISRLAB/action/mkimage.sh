docker stop action_module
docker rm action_module
docker rmi action_image
docker build -t action_image .
docker run -d --name action_module -p 8090:8090 action_image
docker ps
