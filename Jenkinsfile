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
                    dir('dashboard-web-application/client') {

                        script {
                            gv_job_script.build_node()
                            // sh 'echo building'
                        }
                    }
            }
        }

        stage("create frontend docker image") {
            steps {
                script {
                    gv_job_script.create_docker_image()
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
