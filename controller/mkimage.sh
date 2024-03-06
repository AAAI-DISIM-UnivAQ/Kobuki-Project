docker stop controller_module
docker rm controller_module
docker rmi controller_image
docker build -t controller_image .
docker run -d --name controller_module -p 8100:8100 controller_image
docker ps
