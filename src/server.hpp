#pragma once

#include "config.hpp"
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <functional>

namespace auvis {
    class Server {
    public:
        void connect()
        {
            Serial.println();
            Serial.println("Starting server");
            connect_wifi();
        }

        void poll() {
            if(!WiFi.isConnected())
                connect_wifi();
            _server.handleClient();
        }
        
        void listen(short port) {
           _server.begin(port);
            Serial.printf("Listening on port %d\n", port);
        }

        void on(const Uri& uri, std::function<void(ESP8266WebServer&)> handler)
        {
            _server.on(uri, [&, h = std::move(handler)](){ h(_server); });
        }

    private:
        void connect_wifi()
        {
            Serial.printf("Trying to connect to SSID \"%s\"\n", wifi_ssid);
            WiFi.mode(WIFI_STA);
            WiFi.begin(wifi_ssid, wifi_pass);
            while(WiFi.status() == WL_DISCONNECTED)
                delay(100);

            if(!WiFi.isConnected())
            {
                Serial.printf("Error connecting to WiFi. (code %d)\n", int(WiFi.status()));
                delay(2000);
            }
            else
            {
                Serial.print("Successfully connected. IP: ");
                Serial.print(WiFi.localIP());
                Serial.println();
            }
        }

        ESP8266WebServer _server;
    };
}