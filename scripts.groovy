def build_node() {
    echo "building the node application..."

    // sh 'cd dashboard-web-application/client'
    sh 'npm install'
    sh 'npm run build'
} 



def create_docker_image() {
    echo "creating the docker image..."
    withCredentials([usernamePassword(credentialsId: 'docker-hub-cred', passwordVariable: 'PASS', usernameVariable: 'USER')]) {
        sh 'docker build -t docker/eslamdyba/amrs-in-warehouse-systems:dashboard-0.0 .'
        sh "echo $PASS | docker login -u $USER --password-stdin"
        sh 'docker push nanajanashia/demo-app:jma-2.0'
    }

} 

def deploy() {
    echo 'deploying the application...'
} 

return this
