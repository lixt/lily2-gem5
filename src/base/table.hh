/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University
 * All rights reserved.
 */

#ifndef __BASE_TABLE_HH__
#define __BASE_TABLE_HH__

#include <iostream>
#include <utility>
#include <cassert>

template <size_t Set>
class average
{
    size_t operator() (size_t key)
    {
        return key % Set;
    }
};

class error
{
    size_t operator() (size_t set, size_t way)
    {
        std::cout << "Table is full." << std::endl;
        assert (0);
    }
};

template <size_t Set,
          size_t Way,
          class Key,
          class T,
          class Compare = std::less<Key>,
          class Hash = average<Set>,
          class Replace = error>
class Table
{
  public:
    typedef Key key_type;
    typedef T mapped_type;
    typedef std::pair<Key, T> value_type;

  public:
    // Class POSITION separates the outer interfaces and the bottom memory.
    typedef struct Position {
        size_t set; // Belongs to which set.
        size_t way; // Belongs to which way.

        // Print the position in format ``(set,way)''.
        friend std::ostream& operator<< (std::ostream &os, Position position)
        {
            return (os << "(" << position.set << "," << position.way << ")");
        }
    } Position;

    // Returns a nil POSITION.
    Position nil (void) const
    {
        Position nilPos;
        nilPos.set = Set;
        nilPos.way = Way;
        return nilPos;
    }

  public:
    // Accesses the mapped value in POSITION.
    //mapped_value access (const Position &position) const;

    // Searches the KEY in table. On success returns the position. On failure
    // returns a nil position.
    //Position search (const key_type &key) const;

    // Inserts a pair of KEY and MAPPED into the table. If the table is full,
    // then the Replace functor will be called to get rid of an old one. After
    // the insertion, returns the position.
    //Position insert (const Key_type &key, const mapped_type &mapped);

  protected:
    // Traverse the table. Call the F on every entry in the table.
    //void traverse (void (*f) (const key_type &, const mapped_type &));

  protected:


    // Checks the validation of SET.
    bool isSetValid (size_t set) const
    {
        return (set >= 0 && set < Set);
    }

    // Checks the validation of WAY.
    bool isWayValid (size_t way) const
    {
        return (way >= 0 && way < Way);
    }

    // Checks the validation of POSITION.
    bool isPosValid (Position position) const
    {
        return isSetValid (position.set) && isWayValid (position.way);
    }

  private:
    // Type for table entries.
    typedef struct {
        key_type key;
        mapped_type mapped;

        // Auxiliary fields.
        bool valid;
    } entry_type;

  private:
    // Interfaces between bottom pointers and upper positions.
    entry_type* getEntryPtr (Position position)
    {
        if (isPosValid (position)) {
            return &table[position.set][position.way];
        } else {
            //invalidPosFault (position);
        }
    }
    const entry_type* getEntryPtr (Position position) const
    {
        if (isPosValid (position)) {
            return &table[position.set][position.way];
        } else {
            //invalidPosFault (position);
        }
    }

    // Bottom accessor and mutator of table key.
    key_type& entryKey (const entry_type *entryPtr)
    {
        return entryPtr->key;
    }
    const key_type& entryKey (const entry_type *entryPtr) const
    {
        return entryPtr->key;
    }
    void setEntryKey (const entry_type *entryPtr, const key_type &key)
    {
        entryPtr->key = key;
    }

    // Bottom accessor and mutator of table mapped value.
    mapped_type& entryMapped (const entry_type *entryPtr)
    {
        return entryPtr->mapped;
    }
    const mapped_type& entryMapped (const entry_type *entryPtr) const
    {
        return entryPtr->mapped;
    }
    void setEntryMapped (const entry_type *entryPtr, const mapped_type &mapped)
    {
        entryPtr->mapped = mapped;
    }

    // Bottom accessor and mutator of table valid field.
    bool entryValid (const entry_type *entryPtr)
    {
        return entryPtr->valid;
    }
    void setEntryValid (const entry_type *entryPtr, bool valid)
    {
        entryPtr->valid = valid;
    }

  private:
    // Bottom memory model.
    entry_type table[Set][Way];
};

#define TEMPLATE_LIST <size_t Set,    \
                       size_t Way,    \
                       class Key,     \
                       class T,       \
                       class Compare, \
                       class Hash,    \
                       class Replace>
#define TEMPALTE_TABLE Table<Set,     \
                             Way,     \
                             Key,     \
                             T,       \
                             Compare, \
                             Hash,    \
                             Replace>


#endif // __BASE_TABLE_HH__
