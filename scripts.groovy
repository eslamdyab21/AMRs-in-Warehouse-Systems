def install_node_dependencies() {
    echo "installing the node application dependencies..."
    sh 'npm install'
}


def build_node() {
    echo "building the node application..."
    sh 'npm run build'
} 



def create_docker_image() {
    echo "creating the docker image..."
    withCredentials([usernamePassword(credentialsId: 'docker-hub-cred', passwordVariable: 'PASS', usernameVariable: 'USER')]) {
        sh 'docker build -t eslamdyba/amrs-in-warehouse-systems:dashboard-dev-0.0 .'
    }
} 


def push_docker_image() {
    echo "pushing the docker image..."
    withCredentials([usernamePassword(credentialsId: 'docker-hub-cred', passwordVariable: 'PASS', usernameVariable: 'USER')]) {
        sh "echo $PASS | docker login -u $USER --password-stdin"
        sh 'docker push eslamdyba/amrs-in-warehouse-systems:dashboard-dev-0.0'
    }
} 


def deploy() {
    echo 'deploying the application...'
} 

return this
