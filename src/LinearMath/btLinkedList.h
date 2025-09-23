#ifndef BT_LINKEDLIST_H
#define BT_LINKEDLIST_H

template <typename T>
class btLink
{
public:
	btLink() : m_next(0), m_prev(0) {}
	btLink(btLink *next, btLink *prev) : m_next(next), m_prev(prev) {}
	btLink(btLink *next, btLink *prev, T value) : m_next(next), m_prev(prev), m_value(value) {}

	btLink *getNext() const { return m_next; }
	btLink *getPrev() const { return m_prev; }

	T& getValue() { return m_value; }
	const T& getValue() const { return m_value; }
	void setValue(const T& value) { m_value = value; }

	bool isHead() const { return m_prev == 0; }
	bool isTail() const { return m_next == 0; }

	void insertBefore(btLink *link)
	{
		m_next = link;
		m_prev = link->m_prev;
		m_next->m_prev = this;
		m_prev->m_next = this;
	}

	void insertAfter(btLink *link)
	{
		m_next = link->m_next;
		m_prev = link;
		m_next->m_prev = this;
		m_prev->m_next = this;
	}

	void remove()
	{
		m_next->m_prev = m_prev;
		m_prev->m_next = m_next;
	}

private:
	btLink *m_next;
	btLink *m_prev;
	T m_value;
};

template <typename T>
class btLinkedList
{
public:
	btLinkedList() : m_head(&m_tail, 0), m_tail(0, &m_head) {}

	btLink<T> *getHead() const { return m_head.getNext(); }
	btLink<T> *getTail() const { return m_tail.getPrev(); }

	void addHead(btLink<T> *link) { link->insertAfter(&m_head); }
	void addTail(btLink<T> *link) { link->insertBefore(&m_tail); }

	// Create a node from a value and insert it.
	btLink<T>* addHead(const T& value)
	{
		btLink<T>* node = new btLink<T>(); // next/prev set later by insertAfter
		node->setValue(value);
		node->insertAfter(&m_head);
		return node;
	}

	btLink<T>* addTail(const T& value)
	{
		btLink<T>* node = new btLink<T>(); // next/prev set later by insertBefore
		node->setValue(value);
		node->insertBefore(&m_tail);
		return node;
	}

private:
	btLink<T> m_head;
	btLink<T> m_tail;
};

#endif  //BT_LINKEDLIST_H
