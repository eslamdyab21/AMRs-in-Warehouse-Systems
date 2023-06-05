#!/usr/bin/env groovy

// def gv_job_script

pipeline {
    agent any
    tools {
        nodejs '20.2.0'
    }

    stages {
        stage("init") {
            steps {
                script {
                    // gv_job_script = load "scripts.groovy"
                    sh 'echo init'
                }
            }
        }

        stage("build node app") {
            steps {
                script {
                    // gv_job_script.build_node()
                    sh 'echo building'
                }
            }
        }

        stage("build image") {
            steps {
                script {
                    // gv_job_script.build_docker_image()
                    sh 'echo building'
                }
            }
        }

        stage("deploy") {
            steps {
                script {
                    // gv_job_script.deploy()
                    sh 'echo deploying'
                }
            }
        }
    }
}
