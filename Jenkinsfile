#!/usr/bin/env groovy
// See: https://github.com/jenkinsci/pipeline-examples/blob/master/docs/BEST_PRACTICES.md
// See: https://jenkins.io/doc/book/pipeline/
pipeline {
    agent {
        label 'docker'
    }
    options {
        timeout(time: 1, unit: 'HOURS')
        buildDiscarder(logRotator(numToKeepStr: '30', daysToKeepStr: '30'))
        skipDefaultCheckout()
    }
    stages {
        stage("Jenkins environment injection") {
            steps {
                populateEnv()
                script {
                    // We don't want the basebasedev one, instead we want the last drive one that worked.
                    env.BASE_IMAGE = "${env.DOCKER_REPO_HOST_SHORT}:${env.DOCKER_REPO_HOST_PORT}/plusai/drive:latest"
                }
            }
        }
        stage("Cleanup") {
            steps {
                deleteDir()
                sh """
                docker image rm -f ${env.BASE_IMAGE}
                """
            }
        }
        stage("Checkout") {
            steps {
                // See: https://jenkins.io/doc/pipeline/steps/workflow-scm-step/
                script {
                    def scm_vars = checkout scm
                    printMap(scm_vars, "My scm vars are:")
                    printMap(env.getEnvironment(), "My environment is:")
                    env.GIT_COMMIT = scm_vars.GIT_COMMIT
                    env.GIT_BRANCH = scm_vars.GIT_BRANCH
                }
                milestone(ordinal: 0, label: "CHECKOUT_FINISH_MILESTONE")
            }
        }
        stage("Environment setup") {
            steps {
                sh """
                rm -rf ${env.ROS_HOME}
                mkdir ${env.ROS_HOME}
                """
            }
        }
        stage("Building + packaging") {
            steps {
                script {
                    docker.withRegistry("${env.DOCKER_REPO_URL}", "${env.DOCKER_REPO_HOST_CRED}") {
                        runWithImage(env.BASE_IMAGE, "./package.sh")
                    }
                }
            }
        }
        stage("Uploading packages") {
            when {
                branch 'master'
            }
            steps {
                // Upload to our jenkins master so that we can kick a job that will
                // put this in our debian repo for later usage/download/install/whatever.
                script {
                    def am_sent = uploadPackages("${env.WORKSPACE}/packages", "${env.WORKSPACE}/packages")
                    if (am_sent == 0) {
                        throw new RuntimeException("No packages were built (or sent)!")
                    }
                    else {
                        // Kick this job to ensure that the packages get processed sooner than
                        // it's periodic run...
                        build(job: "packer", wait: false)
                    }
                }
            }
        }
    }
}
