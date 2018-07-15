up:
	(cd lyo-webapp-parent && mvn clean package)
	(cd deployment 				&& docker-compose build)
	(cd deployment 				&& docker-compose up)
