#!/usr/bin/env groovy

def gv_job_script

pipeline {
    agent any
    tools {
        nodejs '20.2.0'
    }

    stages {
        stage("init") {
            steps {
                script {
                    sh 'echo init'
                    gv_job_script = load "scripts.groovy"
                }
            }
        }

        stage("install frontend node app dependencies") {
            steps {
                    dir('dashboard-web-application/client') {

                        script {
                            gv_job_script.install_node_dependencies()
                        }
                    }
            }
        }

        stage("build frontend node app") {
            steps {
                    dir('dashboard-web-application/client') {

                        script {
                            gv_job_script.build_node()
                        }
                    }
            }
        }

        stage("create frontend docker image") {
            steps {
                script {
                    gv_job_script.create_docker_image()
                }
            }
        }

        stage("push frontend docker image") {
            steps {
                script {
                    gv_job_script.push_docker_image()
                }
            }
        }

        stage("dev deploy docker frontend") {
            steps {
                script {
                    sh 'echo deploying'
                }
            }
        }
    }
}
