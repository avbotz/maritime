/* messenger.h
 *
 * Very minimal multiple pub-sub data passing implementation. Based heavily on
 * Jacob Dahl's implementation in TeensyFlight36
 * [https://github.com/dakejahl/TeensyFlight36/blob/develop/include/Messenger.hpp]
 *
 * Adapted to allow multiple instances of each messenger data type
 *
 * @author Kalyan Sriram <kalyan@coderkalyan.com>
 * @author Vincent Wang <vwangsf@gmail.com>
 */

#pragma once

#include <vector>

#include <zephyr.h>

namespace messenger {

const int MAX_INSTANCES = 8;

template <typename T> class Publisher;
template <typename T> class Subscriber;
template <typename T> class DataFile;

/* Container for published data. Shared between all publishers and subscribers
 * of a type.
 */
template <typename T> class DataFile
{
public:
	static T &get(int inst)
	{
		return _data[inst];
	}

	static void set_data(int inst, T data)
	{
		_data[inst] = data;
	}

	static void subscribe(int inst, Subscriber<T> *sub)
	{
		_subscribers.push_back(sub);
	}

	static void notify(int inst)
	{
		for (auto &sub : _subscribers) {
			if (sub->inst() == inst) {
				sub->notify();
			}
		}
	}

protected:
	static T _data[MAX_INSTANCES];
	static std::vector<Subscriber<T> *> _subscribers;
};


template <class T> T DataFile<T>::_data[MAX_INSTANCES];
template <class T> std::vector<Subscriber<T> *> DataFile<T>::_subscribers;

template <typename T>
class Subscriber
{
	Subscriber(int inst)
	{
		unsigned int key;
		key = irq_lock();

		_inst = inst;
		_file->subscribe(_inst, this);
		irq_unlock(key);
	}

	Subscriber() : Subscriber(0) {}

	int inst(void)
	{
		return _inst;
	}

	void notify(void)
	{
		_updated = true;
	}

	bool updated(void)
	{
		return _updated;
	}

	T get(void)
	{
		unsigned int key;
		key = irq_lock();

		auto data = _file->get(_inst);
		_updated = false;

		irq_unlock(key);

		return data;
	}

private:
	bool _updated = false;
	DataFile<T> *_file;
	int _inst;
};


template <typename T>
class Publisher
{
public:
	Publisher(int inst)
	{
		_inst = inst;
	}

	void publish(T &data)
	{
		unsigned int key;
		key = irq_lock();
		
		_file->set_data(_inst, data);
		_file->notify(_inst);

		irq_unlock(key);
	}

private:
	DataFile<T> *_file;
	int _inst;
};

}; /* namespace messenger */
