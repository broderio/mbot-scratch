#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <iostream>
#include <string>

#include "server.hpp"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

Server::Server(std::string address, int port, Server::SessionHandler handler)
: acceptor_(ioc_, tcp::endpoint(net::ip::make_address(address), port)), signals_(ioc_, SIGINT, SIGTERM), handler_(handler) {
        accept();
        waitForSignal();
}

void Server::run() {
    ioc_.run();
}

void Server::accept() {
    acceptor_.async_accept(
        [this](beast::error_code ec, tcp::socket socket) {
            if (ec) {
                std::cerr << "Accept error: " << ec.message() << std::endl;
            } else {
                std::make_shared<Session>(std::move(socket), handler_)->run();
            }

            accept();
        });
}

void Server::waitForSignal() {
    signals_.async_wait(
        [this](beast::error_code ec, int signal) {
            if (ec) {
                std::cerr << "Signal error: " << ec.message() << std::endl;
            } else {
                ioc_.stop();
            }
        });
}

Server::Session::Session(tcp::socket socket, Server::SessionHandler handler)
: ws_(std::move(socket)), handler_(handler) {
}

void Server::Session::run() {
    handler_(ws_);
}