#!/usr/bin/env groovy

def gv_job_script

pipeline {
    agent any
    tools {
        NodeJS '20.2.0'
    }

    stages {
        stage("init") {
            steps {
                script {
                    gv_job_script = load "scripts.groovy"
                }
            }
        }

        stage("build node app") {
            steps {
                script {
                    gv_job_script.build_node()
                }
            }
        }

        stage("build image") {
            steps {
                script {
                    gv_job_script.build_docker_image()
                }
            }
        }

        stage("deploy") {
            steps {
                script {
                    gv_job_script.deploy()
                }
            }
        }
    }
}
