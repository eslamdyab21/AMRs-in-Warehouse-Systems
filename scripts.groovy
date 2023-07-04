def create_docker_image() {
    echo "creating the docker image..."
    withCredentials([usernamePassword(credentialsId: 'docker-hub-cred', passwordVariable: 'PASS', usernameVariable: 'USER')]) {
        sh 'docker build -t eslamdyba/amrs-in-warehouse-systems:postgressql-server-dev-1.0 --build-arg POSTGRES_USER=$POSTGRES_USER --build-arg POSTGRES_DB=$POSTGRES_DATABASE --build-arg POSTGRES_PASSWORD=$POSTGRES_PASSWORD -f docker-with-databases/Dockerfile .'
    }
} 


def push_docker_image() {
    echo "pushing the docker image..."
    withCredentials([usernamePassword(credentialsId: 'docker-hub-cred', passwordVariable: 'PASS', usernameVariable: 'USER')]) {
        sh "echo $PASS | docker login -u $USER --password-stdin"
        sh 'docker push eslamdyba/amrs-in-warehouse-systems:postgressql-server-dev-1.0'
    }
} 


def deploy() {
    echo 'deploying the application...'
} 

return this