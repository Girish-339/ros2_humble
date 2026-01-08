pipeline {
    agent any

    stages {
        stage('Checkout') {
            steps {
                checkout scm
            }
        }

        stage('Build Docker Image') {
            steps {
                sh 'docker build -t ros2-humble-ci .'
            }
        }

        stage('Run Container') {
            steps {
                sh 'docker run --rm ros2-humble-ci echo "ROS2 Build OK"'
            }
        }
    }
}
