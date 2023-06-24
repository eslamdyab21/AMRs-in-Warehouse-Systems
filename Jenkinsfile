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

        stage("install backend node app dependencies") {
            steps {
                    dir('dashboard-web-application/server') {

                        script {
                            gv_job_script.install_node_dependencies()
                        }
                    }
            }
        }

        stage("create backend docker image") {
            steps {
                script {
                    gv_job_script.create_docker_image()
                }
            }
        }

        stage("push backend docker image") {
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