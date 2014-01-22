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

        // Constructors.
        Position (void) : set (0), way (0) {}
        Position (size_t set, size_t way) : set (set), way (way) {}

        // Overloads relationship operator ``==''.
        bool operator== (const Position &position)
        {
            return (this->set == position.set && this->way == position.way);
        }

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
    mapped_type access (const Position &position) const;

    // Searches the KEY in table. On success returns the position. On failure
    // returns a nil position.
    Position search (const key_type &key) const;

    // Inserts a pair of KEY and MAPPED into the table. If the table is full,
    // then the Replace functor will be called to get rid of an old one. After
    // the insertion, returns the position.
    Position insert (const key_type &key, const mapped_type &mapped);

  protected:
    // Traverse the table. Call the F on every entry in the table.
    //void traverse (void (*f) (const key_type &, const mapped_type &));

  protected:
    // Informs an invalid position fault and force quits the program.
    void invalidPosFault (const Position &position) const
    {
        std::cerr << "Invalid position range fault occurs."
                  << "Fault position: " << position << std::endl;
        assert (0);
    }

    // Informs an invalid entry fault and force quits the program.
    void invalidEntryFault (const Position &position) const
    {
        std::cerr << "Invalid position fault occurs."
                  << "Fault position: " << position << std::endl;
        assert (0);
    }

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

    // Checks the range validation of POSITION.
    bool isPosRangeValid (const Position &position) const
    {
        return isSetValid (position.set) && isWayValid (position.way);
    }

    // Checks the validation of POSITION.
    bool isPosValid (const Position &position) const
    {
        if (isPosRangeValid (position)) {
            return entryValid (position);
        } else {
            return false;
        }
    }

  protected:
    // Upper accessor and mutator of the table key.
    key_type &entryKey (const Position &position)
    {
        if (!isPosValid (position)) {
            invalidPosFault (position);
        }
        return entryKey (getEntryPtr (position));
    }
    const key_type &entryKey (const Position &position) const
    {
        if (!isPosValid (position)) {
            invalidPosFault (position);
        }
        return entryKey (getEntryPtr (position));
    }
    void setEntryKey (const Position &position, const key_type &key)
    {
        if (!isPosValid (position)) {
            invalidPosFault (position);
        }
        setEntryKey (getEntryPtr (position), key);
    }

    // Upper accessor and mutator of the table mapped value.
    mapped_type &entryMapped (const Position &position)
    {
        if (!isPosValid (position)) {
            invalidPosFault (position);
        }
        return entryMapped (getEntryPtr (position));
    }
    const mapped_type &entryMapped (const Position &position) const
    {
        if (!isPosValid (position)) {
            invalidPosFault (position);
        }
        return entryMapped (getEntryPtr (position));
    }
    void setEntryMapped (const Position &position, const mapped_type &mapped)
    {
        if (!isPosValid (position)) {
            invalidPosFault (position);
        }
        setEntryMapped (getEntryPtr (position), mapped);
    }

    // Upper accessor and mutator of the table valid field.
    bool entryValid (const Position &position) const
    {
        if (!isPosRangeValid (position)) {
            invalidPosRangeFault (position);
        }
        return entryValid (getEntryPtr (position));
    }
    void setEntryValid (const Position &position, bool valid)
    {
        if (!isPosRangeValid (position)) {
            invalidPosRangeFault (position);
        }
        setEntryValid (getEntryPtr (position), valid);
    }

  protected:
    // Finds a vacant entry in table.
    Position findVacantPos (void) const
    {
        for (int i = 0; i != Set; ++i) {
            Position curPos = findVacantPosInSet (i);
            if (curPos == nil ()) {
                continue;
            } else {
                return curPos;
            }
        }

        // Returns nil if no vacant entry is found.
        return nil ();
    }

    // Finds a vacant entry in a set.
    Position findVacantPosInSet (size_t set) const
    {
        for (int j = 0; j != Way; ++j) {
            Position curPos = Position (set, j);
            if (isPosValid (curPos)) {
                continue;
            } else {
                return curPos;
            }
        }

        // Returns nil if no vacant entry is found.
        return nil ();
    }

    // Inserts contents into a given entry.
    Position insert (const Position &position,
                     const key_type &key, const mapped_type &mapped)
    {
        setEntryValid (position, true);
        setEntryKey (position, key);
        setEntryMapped (position, mapped);

        return position;
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
    entry_type* getEntryPtr (const Position &position)
    {
        entry_type *retval;

        if (isPosValid (position)) {
            retval =  &table[position.set][position.way];
        } else {
            retval = NULL;
        }

        return retval;
    }
    const entry_type* getEntryPtr (const Position &position) const
    {
        entry_type *retval;

        if (isPosValid (position)) {
            retval = &table[position.set][position.way];
        } else {
            retval = NULL;
        }

        return retval;
    }

    // Bottom accessor and mutator of the table key.
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

    // Bottom accessor and mutator of the table mapped value.
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

    // Bottom accessor and mutator of the table valid field.
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

#define TEMPLATE_LIST template<size_t Set,    \
                               size_t Way,    \
                               class Key,     \
                               class T,       \
                               class Compare, \
                               class Hash,    \
                               class Replace>
#define TEMPLATE_TABLE Table<Set,     \
                             Way,     \
                             Key,     \
                             T,       \
                             Compare, \
                             Hash,    \
                             Replace>
TEMPLATE_LIST
typename TEMPLATE_TABLE::mapped_type
TEMPLATE_TABLE::access (const Position &position) const
{
    if (!isPosValid (position)) {
        invalidPosFault (position);
    }

    return entryMapped (position);
}

TEMPLATE_LIST
typename TEMPLATE_TABLE::Position
TEMPLATE_TABLE::search (const key_type &key) const
{
    size_t set = Hash (key) % Set;

    for (int wi = 0; wi != Way; ++wi) {
        Position curPos = Position (set, wi);

        if (!isPosValid (curPos)) {
            continue;
        } else {
            if (entryKey (curPos) != key) {
                continue;
            } else {
                // Returns the first found key.
                return curPos;
            }
        }
    }

    // If KEY is not found, returns a nil position.
    return nil ();
}

TEMPLATE_LIST
typename TEMPLATE_TABLE::Position
TEMPLATE_TABLE::insert (const key_type &key, const mapped_type &mapped)
{
    size_t set = Hash (key) % Set;

    Position insertPos = findVacantPosInSet (set);
    if (insertPos == nil ()) {
        // Table is full, calls the Replace functor.
        insertPos.set = set;
        insertPos.way = Replace (set);
    }

    insert (insertPos, key, mapped);
}

#endif // __BASE_TABLE_HH__
