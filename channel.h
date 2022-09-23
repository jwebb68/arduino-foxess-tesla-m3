// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2 -*-
#if !defined(CHANNEL_H)
#    define CHANNEL_H

template<typename T>
class Sender
{
    public:
        // send v to whereever,
        // returns true if sent ok
        // returns false if not sent
        // returns false if channel closed. (?)
        virtual bool send(T &&v) = 0;
};

template<typename T>
class Receiver
{
    public:
        // recv a T from whereever
        // returns true if v contains a valid value
        // returns false if channel closed (?)
        virtual bool recv(T &v) = 0;
};

class FlushedIO
{
    public:
        virtual int write(uint8_t const arr[], size_t len) = 0;
        virtual int write(uint8_t v) = 0;
        virtual void flush_output(void) = 0;

        virtual int in_waiting(void) = 0;
        virtual int read(uint8_t arr[], size_t len) = 0;
        virtual int read(uint8_t &v) = 0;
        virtual void flush_input(void) = 0;
};

#endif // defined(CHANNEL_H)
