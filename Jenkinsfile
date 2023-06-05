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

        stage("build frontend node app") {
            steps {
                    sh "pwd"
                    dir('dashboard-web-application/client') {
                        sh "pwd"

                        script {
                            gv_job_script.build_node()
                            // sh 'echo building'
                        }
                    }
                    sh "pwd"
            }
        }

        stage("build frontend docker image") {
            steps {
                script {
                    gv_job_script.build_docker_image()
                    // sh 'echo building'
                }
            }
        }

        stage("deploy docker frontend") {
            steps {
                script {
                    // gv_job_script.deploy()
                    sh 'echo deploying'
                }
            }
        }
    }
}
