#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define TRIG 6  // wiringPi pin 6 = BCM GPIO 25
#define ECHO 2  // wiringPi pin 2 = BCM GPIO 27
#define PORT 5001
#define SERVER_IP "127.0.0.1"

long get_microseconds() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

float measure_distance() {
    long start_time, end_time;

    // Send 10µs pulse
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    // Wait for echo to start
    while (digitalRead(ECHO) == LOW);
    start_time = get_microseconds();

    // Wait for echo to end
    while (digitalRead(ECHO) == HIGH);
    end_time = get_microseconds();

    return (end_time - start_time) / 58.0;
}

int main() {
    int client_fd;
    struct sockaddr_in serv_addr;
    char message[64];

    // Setup wiringPi
    if (wiringPiSetup() == -1) {
        printf("wiringPi setup failed\n");
        return 1;
    }

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    digitalWrite(TRIG, LOW);
    delay(500);

    // Create socket
    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation failed");
        return 1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        perror("Invalid address");
        return 1;
    }

    if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection failed");
        return 1;
    }

    printf("Connected to server at %s:%d\n", SERVER_IP, PORT);

    while (1) {
        float distance = measure_distance();
        snprintf(message, sizeof(message), "Distance: %.2f cm\n", distance);

        send(client_fd, message, strlen(message), 0);
        printf("Sent: %s", message);
        char buffer[64];
        recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        printf("Line Follower: %s\n", buffer);

        delay(50);  // 50ms interval
    }

    close(client_fd);
    return 0;
}