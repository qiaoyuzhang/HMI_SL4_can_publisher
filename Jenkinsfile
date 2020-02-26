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
                script {
                    env.ROS_HOME = "${env.WORKSPACE}/.ros/"
                }
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
                        def img = docker.image(env.BASE_IMAGE)
                        img.inside {
                            sh """
                            # Get a newer rosdep, the ubuntu one seems busted...
                            virtualenv --system-site-packages .venv
                            . .venv/bin/activate
                            pip install rosdep -I
                            # ./package.sh
                            """
                        }
                    }
                }
            }
        }
    }
}
