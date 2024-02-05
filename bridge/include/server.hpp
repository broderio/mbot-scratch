#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/signal_set.hpp>
#include <iostream>
#include <string>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

class Server {
public:
    using SessionHandler = std::function<void(websocket::stream<tcp::socket>&)>;
    Server(std::string address, int port, SessionHandler handler);
    void run();

private:
    void accept();
    void waitForSignal();

    class Session : public std::enable_shared_from_this<Session> {
    public:
        Session(tcp::socket socket, SessionHandler handler);
        void run();

    private:
        websocket::stream<tcp::socket> ws_;
        beast::flat_buffer buffer_;
        SessionHandler handler_;
    };

    net::io_context ioc_;
    tcp::acceptor acceptor_;
    SessionHandler handler_;
    boost::asio::signal_set signals_;
};