def build_node() {
    echo "building the node application..."

    sh 'cd dashboard-web-application/client'
    sh 'npm install'
    sh 'npm run build'
} 



def build_docker_image() {
    echo "building the docker image..."
} 

def deploy() {
    echo 'deploying the application...'
} 

return this
