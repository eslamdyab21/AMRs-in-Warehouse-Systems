def install_node_dependencies() {
    echo "installing the node application dependencies..."
    sh 'npm install'
}



def create_docker_image() {
    echo "creating the docker image..."
    withCredentials([usernamePassword(credentialsId: 'docker-hub-cred', passwordVariable: 'PASS', usernameVariable: 'USER')]) {
        sh 'docker build -t eslamdyba/amrs-in-warehouse-systems:dashboard-backend-dev-0.0 --build-arg POSTGRES_HOST=$POSTGRES_HOST --build-arg POSTGRES_USER=$POSTGRES_USER --build-arg POSTGRES_DATABASE=$POSTGRES_DATABASE --build-arg POSTGRES_PORT=$POSTGRES_PORT --build-arg POSTGRES_PASSWORD=$POSTGRES_PASSWORD .'
    }
} 


def push_docker_image() {
    echo "pushing the docker image..."
    withCredentials([usernamePassword(credentialsId: 'docker-hub-cred', passwordVariable: 'PASS', usernameVariable: 'USER')]) {
        sh "echo $PASS | docker login -u $USER --password-stdin"
        sh 'docker push eslamdyba/amrs-in-warehouse-systems:dashboard-backend-dev-0.0'
    }
} 


def deploy() {
    echo 'deploying the application...'
} 

return this