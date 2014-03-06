/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University
 * All rights reserved.
 */

#ifndef __BASE_TABLE_HH__
#define __BASE_TABLE_HH__

#include <iostream>
#include <utility>
#include <cassert>
#include <vector>
#include "macro.hh"

template <class T>
struct average
{
    size_t operator() (const T& x) { return static_cast<size_t> (x); }
};

struct error
{
    size_t operator() (size_t set) { assert (0); }
};

template <size_t Set,
          size_t Way,
          class Key,
          class T,
          class Compare = std::less<Key>,
          class Hash = average<Key>,
          class Replace = error>
class Table
{
  public:
    typedef Key key_type;
    typedef T mapped_type;
    typedef std::pair<Key, T> value_type;
    typedef Compare compare_type;
    typedef Hash hash_type;
    typedef Replace replace_type;

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
        return Position (Set, Way);
    }

  public:
    Table (void) { clear (); }

  public:
    // Functions start with ``cb'' will call the callback functions in the
    // Replace funtor and update the replace policy data.

    // Accesses the mapped value according to the given POSITION.
    // If the POSITION is invalid, the function will be aborted.
    mapped_type access (const Position &position) const;
    mapped_type cbAccess (const Position &position) const;

    // Searches the KEY in table. On success returns the position. On failure
    // returns a nil position.
    Position search (const key_type &key) const;
    Position cbSearch (const key_type &key) const;

    // Mutates a new MAPPED value according to the given POSITION.
    // If the POSITION is invalid, the function will be aborted.
    void mutate (const Position &position, const mapped_type &mapped);
    void cbMutate (const Position &position, const mapped_type &mapped);

    // Inserts a pair of KEY and MAPPED into the table. If the table is full,
    // then the Replace functor will be called to get rid of an old one. After
    // the insertion, returns the position.
    Position insert (const key_type &key, const mapped_type &mapped);
    Position cbInsert (const key_type &key, const mapped_type &mapped);

    void remove (const Position& position);
    void cbRemove (const Position& position);

    void clear (void);

  protected:
    // NULL functor that do nothing.
    //class NullFunctor
    //{
    //  public:
    //    bool operator() (key_type &key, mapped_type &mapped)
    //    {
    //        return false;
    //    }
    //};

    //static NullFunctor nfunc;

    // Traverses the table, calls VFUNC on every valid entry and IFUNC on every
    // invalid entry, returns nothing.
    // VFUNC and IFUNC should take key_type& and mapped_type& as the types of the
    // two parameters.
    template <class ValidFunctor>
    void traverse (ValidFunctor& vfunc);
    template <class ValidFunctor>
    void traverse (ValidFunctor& vfunc) const;
    template <class ValidFunctor, class InvalidFunctor>
    void traverse (ValidFunctor& vfunc, InvalidFunctor& ifunc);
    template <class ValidFunctor, class InvalidFunctor>
    void traverse (ValidFunctor& vfunc, InvalidFunctor& ifunc) const;

    // Traverses the table, calls VFUNC on every valid entry and IFUNC on every
    // invalid entry, returns a vector of positions.
    // VFUNC and IFUNC should take key_type& and mapped_type& as the types of the
    // two parameters and returns a bool variable. TRUE indicates the current
    // position will be returned and vice versa.
    template <class ValidFunctor>
    std::vector<Position> traverseAndReturn (ValidFunctor& vfunc);
    template <class ValidFunctor>
    std::vector<Position> traverseAndReturn (ValidFunctor& vfunc) const;
    template <class ValidFunctor, class InvalidFunctor>
    std::vector<Position> traverseAndReturn (ValidFunctor& vfunc, InvalidFunctor& ifunc);
    template <class ValidFunctor, class InvalidFunctor>
    std::vector<Position> traverseAndReturn (ValidFunctor& vfunc, InvalidFunctor& ifunc) const;

    // Traverses a set of the table, calls VFUNC on every valid entry and IFUNC on
    // every invalid entry, returns nothing.
    // VFUNC and IFUNC should take key_type& and mapped_type& as the types of the
    // two parameters.
    template <class ValidFunctor>
    void traverseSet (size_t set, ValidFunctor& vfunc);
    template <class ValidFunctor>
    void traverseSet (size_t set, ValidFunctor& vfunc) const;
    template <class ValidFunctor, class InvalidFunctor>
    void traverseSet (size_t set, ValidFunctor& vfunc, InvalidFunctor& ifunc);
    template <class ValidFunctor, class InvalidFunctor>
    void traverseSet (size_t set, ValidFunctor& vfunc, InvalidFunctor& ifunc) const;

    // Traverses a set of the table, calls VFUNC on every valid entry and IFUNC on
    // every invalid entry, returns a vector of positions.
    // VFUNC and IFUNC should take key_type& and mapped_type& as the types of the
    // two parameters and returns a bool variable. TRUE indicates the current
    // position will be returned and vice versa.
    template <class ValidFunctor>
    std::vector<Position> traverseSetAndReturn (size_t set, ValidFunctor& vfunc);
    template <class ValidFunctor>
    std::vector<Position> traverseSetAndReturn (size_t set, ValidFunctor& vfunc) const;
    template <class ValidFunctor, class InvalidFunctor>
    std::vector<Position> traverseSetAndReturn (size_t set, ValidFunctor& vfunc, InvalidFunctor& ifunc);
    template <class ValidFunctor, class InvalidFunctor>
    std::vector<Position> traverseSetAndReturn (size_t set, ValidFunctor& vfunc, InvalidFunctor& ifunc) const;

    // Prints out readable debug information of the table, calls PFUNC on every
    // entry, returns nothing.
    // PFUNC should take std::ostream&, const key_type& and const mapped_type& as the
    // types of the three paramters and returns nothing.
    template <class PrintFunctor>
    void print (std::ostream &os, PrintFunctor& pfunc) const;

  protected:
    // Informs an invalid position range fault and force quits the program.
    void invalidPosRangeFault (const Position &position) const
    {
        std::cerr << "Invalid position range fault occurs."
                  << "Fault position: " << position << std::endl;
        assert (0);
    }

    // Informs an invalid position fault and force quits the program.
    void invalidPosFault (const Position &position) const
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
        if (!isPosRangeValid (position)) {
            invalidPosRangeFault (position);
        }
        return entryKey (getEntryPtr (position));
    }
    const key_type &entryKey (const Position &position) const
    {
        if (!isPosRangeValid (position)) {
            invalidPosRangeFault (position);
        }
        return entryKey (getEntryPtr (position));
    }
    void setEntryKey (const Position &position, const key_type &key)
    {
        if (!isPosRangeValid (position)) {
            invalidPosRangeFault (position);
        }
        setEntryKey (getEntryPtr (position), key);
    }

    // Upper accessor and mutator of the table mapped value.
    mapped_type &entryMapped (const Position &position)
    {
        if (!isPosRangeValid (position)) {
            invalidPosRangeFault (position);
        }
        return entryMapped (getEntryPtr (position));
    }
    const mapped_type &entryMapped (const Position &position) const
    {
        if (!isPosRangeValid (position)) {
            invalidPosRangeFault (position);
        }
        return entryMapped (getEntryPtr (position));
    }
    void setEntryMapped (const Position &position, const mapped_type &mapped)
    {
        if (!isPosRangeValid (position)) {
            invalidPosRangeFault (position);
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
            if (curPos != nil ()) {
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
            Position curPos (set, j);
            if (!isPosValid (curPos)) {
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

        if (isPosRangeValid (position)) {
            retval = &table[position.set][position.way];
        } else {
            retval = NULL;
        }

        return retval;
    }
    const entry_type* getEntryPtr (const Position &position) const
    {
        const entry_type *retval;

        if (isPosRangeValid (position)) {
            retval = &table[position.set][position.way];
        } else {
            retval = NULL;
        }

        return retval;
    }

    // Bottom accessor and mutator of the table key.
    key_type& entryKey (entry_type *entryPtr)
    {
        return entryPtr->key;
    }
    const key_type& entryKey (const entry_type *entryPtr) const
    {
        return entryPtr->key;
    }
    void setEntryKey (entry_type *entryPtr, const key_type &key)
    {
        entryPtr->key = key;
    }

    // Bottom accessor and mutator of the table mapped value.
    mapped_type& entryMapped (entry_type *entryPtr)
    {
        return entryPtr->mapped;
    }
    const mapped_type& entryMapped (const entry_type *entryPtr) const
    {
        return entryPtr->mapped;
    }
    void setEntryMapped (entry_type *entryPtr, const mapped_type &mapped)
    {
        entryPtr->mapped = mapped;
    }

    // Bottom accessor and mutator of the table valid field.
    bool entryValid (const entry_type *entryPtr) const
    {
        return entryPtr->valid;
    }
    void setEntryValid (entry_type *entryPtr, bool valid)
    {
        entryPtr->valid = valid;
    }

  private:
    // Functors.
    compare_type compare;
    hash_type hash;
    replace_type replace;

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
#define TEMPLATE_CLASS Table<Set,     \
                             Way,     \
                             Key,     \
                             T,       \
                             Compare, \
                             Hash,    \
                             Replace>

TEMPLATE_LIST
typename TEMPLATE_CLASS::mapped_type
TEMPLATE_CLASS::access (const Position &position) const
{
    if (!isPosValid (position)) {
        invalidPosFault (position);
    }

    return entryMapped (position);
}

TEMPLATE_LIST
typename TEMPLATE_CLASS::mapped_type
TEMPLATE_CLASS::cbAccess (const Position &position) const
{
    if (!isPosValid (position)) {
        invalidPosFault (position);
    }

    replace.accessCallback (position.set, position.way);
    return access (position);
}

TEMPLATE_LIST
typename TEMPLATE_CLASS::Position
TEMPLATE_CLASS::search (const key_type &key) const
{
    Hash searchHash;
    size_t set = searchHash (key) % Set;

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
typename TEMPLATE_CLASS::Position
TEMPLATE_CLASS::cbSearch (const key_type &key) const
{
    Position searchPos = search (key);

    if (searchPos != nil ()) {
        replace.searchCallback (searchPos.set, searchPos.way);
    }

    return searchPos;
}

TEMPLATE_LIST
void
TEMPLATE_CLASS::mutate (const Position &mutatePos, const mapped_type &mapped)
{
    if (!isPosValid (mutatePos)) {
        invalidPosFault (mutatePos);
    }

    setEntryMapped (mutatePos, mapped);
}

TEMPLATE_LIST
void
TEMPLATE_CLASS::cbMutate (const Position &mutatePos, const mapped_type &mapped)
{
    if (!isPosValid (mutatePos)) {
        invalidPosFault (mutatePos);
    }

    replace.mutateCallback (mutatePos.set, mutatePos.way);
    setEntryMapped (mutatePos, mapped);
}

TEMPLATE_LIST
typename TEMPLATE_CLASS::Position
TEMPLATE_CLASS::insert (const key_type &key, const mapped_type &mapped)
{
    size_t set = hash (key) % Set;

    Position insertPos = findVacantPosInSet (set);
    if (insertPos == nil ()) {
        insertPos.set = set;
        insertPos.way = replace (set);
    }

    return insert (insertPos, key, mapped);
}

TEMPLATE_LIST
typename TEMPLATE_CLASS::Position
TEMPLATE_CLASS::cbInsert (const key_type &key, const mapped_type &mapped)
{
    Position insertPos = insert (key, mapped);

    if (insertPos != nil ()) {
        replace.insertCallback (insertPos.set, insertPos.way);
    }

    return insertPos;
}

TEMPLATE_LIST
void
TEMPLATE_CLASS::remove (const Position& removePos)
{
    if (!isPosRangeValid (removePos)) {
        invalidPosRangeFault (removePos);
    }

    setEntryValid (removePos, false);
}

TEMPLATE_LIST
void
TEMPLATE_CLASS::cbRemove (const Position& removePos)
{
    if (!isPosRangeValid (removePos)) {
        invalidPosRangeFault (removePos);
    }

    setEntryValid (removePos, false);
    replace.removeCallback (removePos.set, removePos.way);
}

TEMPLATE_LIST
void
TEMPLATE_CLASS::clear (void)
{
    for (size_t i = 0; i != Set; ++i) {
        for (size_t j = 0; j != Way; ++j) {
            Position curPos (i, j);
            remove (curPos);
        }
    }
}

TEMPLATE_LIST
template <class ValidFunctor>
void
TEMPLATE_CLASS::traverse (ValidFunctor& vfunc)
{
    for (int i = 0; i != Set; ++i) {
        for (int j = 0; j != Way; ++j) {
            Position curPos (i, j);

            if (isPosValid (curPos)) {
                vfunc (entryKey (curPos), entryMapped (curPos));
            } else {
                ;
            }
        }
    }
}

TEMPLATE_LIST
template <class ValidFunctor>
void
TEMPLATE_CLASS::traverse (ValidFunctor& vfunc) const
{
    for (int i = 0; i != Set; ++i) {
        for (int j = 0; j != Way; ++j) {
            Position curPos (i, j);

            if (isPosValid (curPos)) {
                vfunc (entryKey (curPos), entryMapped (curPos));
            } else {
                ;
            }
        }
    }
}

TEMPLATE_LIST
template <class ValidFunctor, class InvalidFunctor>
void
TEMPLATE_CLASS::traverse (ValidFunctor& vfunc, InvalidFunctor& ifunc)
{
    for (int i = 0; i != Set; ++i) {
        for (int j = 0; j != Way; ++j) {
            Position curPos (i, j);

            if (isPosValid (curPos)) {
                vfunc (entryKey (curPos), entryMapped (curPos));
            } else {
                ifunc (entryKey (curPos), entryMapped (curPos));
            }
        }
    }
}

TEMPLATE_LIST
template <class ValidFunctor, class InvalidFunctor>
void
TEMPLATE_CLASS::traverse (ValidFunctor& vfunc, InvalidFunctor& ifunc) const
{
    for (int i = 0; i != Set; ++i) {
        for (int j = 0; j != Way; ++j) {
            Position curPos (i, j);

            if (isPosValid (curPos)) {
                vfunc (entryKey (curPos), entryMapped (curPos));
            } else {
                ifunc (entryKey (curPos), entryMapped (curPos));
            }
        }
    }
}

TEMPLATE_LIST
template <class ValidFunctor>
std::vector<typename TEMPLATE_CLASS::Position>
TEMPLATE_CLASS::traverseAndReturn (ValidFunctor& vfunc)
{
    std::vector<Position> vecPos;

    for (int i = 0; i != Set; ++i) {
        for (int j = 0; j != Way; ++j) {
            Position curPos (i, j);

            if (isPosValid (curPos)) {
                if (vfunc (entryKey (curPos), entryMapped (curPos))) {
                    vecPos.push_back (curPos);
                }
            } else {
                ;
            }
        }
    }

    return vecPos;
}

TEMPLATE_LIST
template <class ValidFunctor>
std::vector<typename TEMPLATE_CLASS::Position>
TEMPLATE_CLASS::traverseAndReturn (ValidFunctor& vfunc) const
{
    std::vector<Position> vecPos;

    for (int i = 0; i != Set; ++i) {
        for (int j = 0; j != Way; ++j) {
            Position curPos (i, j);

            if (isPosValid (curPos)) {
                if (vfunc (entryKey (curPos), entryMapped (curPos))) {
                    vecPos.push_back (curPos);
                }
            } else {
                ;
            }
        }
    }

    return vecPos;
}

TEMPLATE_LIST
template <class ValidFunctor, class InvalidFunctor>
std::vector<typename TEMPLATE_CLASS::Position>
TEMPLATE_CLASS::traverseAndReturn (ValidFunctor& vfunc, InvalidFunctor& ifunc)
{
    std::vector<Position> vecPos;

    for (int i = 0; i != Set; ++i) {
        for (int j = 0; j != Way; ++j) {
            Position curPos (i, j);

            if (isPosValid (curPos)) {
                if (vfunc (entryKey (curPos), entryMapped (curPos))) {
                    vecPos.push_back (curPos);
                }
            } else {
                if (ifunc (entryKey (curPos), entryMapped (curPos))) {
                    vecPos.push_back (curPos);
                }
            }
        }
    }

    return vecPos;
}

TEMPLATE_LIST
template <class ValidFunctor, class InvalidFunctor>
std::vector<typename TEMPLATE_CLASS::Position>
TEMPLATE_CLASS::traverseAndReturn (ValidFunctor& vfunc, InvalidFunctor& ifunc) const
{
    std::vector<Position> vecPos;

    for (int i = 0; i != Set; ++i) {
        for (int j = 0; j != Way; ++j) {
            Position curPos (i, j);

            if (isPosValid (curPos)) {
                if (vfunc (entryKey (curPos), entryMapped (curPos))) {
                    vecPos.push_back (curPos);
                }
            } else {
                if (ifunc (entryKey (curPos), entryMapped (curPos))) {
                    vecPos.push_back (curPos);
                }
            }
        }
    }

    return vecPos;
}

TEMPLATE_LIST
template <class ValidFunctor>
void
TEMPLATE_CLASS::traverseSet (size_t set, ValidFunctor& vfunc)
{
    for (int j = 0; j != Way; ++j) {
        Position curPos (set, j);

        if (isPosValid (curPos)) {
            vfunc (entryKey (curPos), entryMapped (curPos));
        } else {
            ;
        }
    }
}

TEMPLATE_LIST
template <class ValidFunctor>
void
TEMPLATE_CLASS::traverseSet (size_t set, ValidFunctor& vfunc) const
{
    for (int j = 0; j != Way; ++j) {
        Position curPos (set, j);

        if (isPosValid (curPos)) {
            vfunc (entryKey (curPos), entryMapped (curPos));
        } else {
            ;
        }
    }
}

TEMPLATE_LIST
template <class ValidFunctor, class InvalidFunctor>
void
TEMPLATE_CLASS::traverseSet (size_t set, ValidFunctor& vfunc, InvalidFunctor& ifunc)
{
    for (int j = 0; j != Way; ++j) {
        Position curPos (set, j);

        if (isPosValid (curPos)) {
            vfunc (entryKey (curPos), entryMapped (curPos));
        } else {
            ifunc (entryKey (curPos), entryMapped (curPos));
        }
    }
}

TEMPLATE_LIST
template <class ValidFunctor, class InvalidFunctor>
void
TEMPLATE_CLASS::traverseSet (size_t set, ValidFunctor& vfunc, InvalidFunctor& ifunc) const
{
    for (int j = 0; j != Way; ++j) {
        Position curPos (set, j);

        if (isPosValid (curPos)) {
            vfunc (entryKey (curPos), entryMapped (curPos));
        } else {
            ifunc (entryKey (curPos), entryMapped (curPos));
        }
    }
}

TEMPLATE_LIST
template <class ValidFunctor>
std::vector<typename TEMPLATE_CLASS::Position>
TEMPLATE_CLASS::traverseSetAndReturn (size_t set, ValidFunctor& vfunc)
{
    std::vector<Position> vecPos;

    for (int j = 0; j != Way; ++j) {
        Position curPos (set, j);

        if (isPosValid (curPos)) {
            if (vfunc (entryKey (curPos), entryMapped (curPos))) {
                vecPos.push_back (curPos);
            }
        } else {
            ;
        }
    }

    return vecPos;
}

TEMPLATE_LIST
template <class ValidFunctor>
std::vector<typename TEMPLATE_CLASS::Position>
TEMPLATE_CLASS::traverseSetAndReturn (size_t set, ValidFunctor& vfunc) const
{
    std::vector<Position> vecPos;

    for (int j = 0; j != Way; ++j) {
        Position curPos (set, j);

        if (isPosValid (curPos)) {
            if (vfunc (entryKey (curPos), entryMapped (curPos))) {
                vecPos.push_back (curPos);
            }
        } else {
            ;
        }
    }

    return vecPos;
}

TEMPLATE_LIST
template <class ValidFunctor, class InvalidFunctor>
std::vector<typename TEMPLATE_CLASS::Position>
TEMPLATE_CLASS::traverseSetAndReturn (size_t set, ValidFunctor& vfunc, InvalidFunctor& ifunc)
{
    std::vector<Position> vecPos;

    for (int j = 0; j != Way; ++j) {
        Position curPos (set, j);

        if (isPosValid (curPos)) {
            if (vfunc (entryKey (curPos), entryMapped (curPos))) {
                vecPos.push_back (curPos);
            }
        } else {
            if (ifunc (entryKey (curPos), entryMapped (curPos))) {
                vecPos.push_back (curPos);
            }
        }
    }

    return vecPos;
}

TEMPLATE_LIST
template <class ValidFunctor, class InvalidFunctor>
std::vector<typename TEMPLATE_CLASS::Position>
TEMPLATE_CLASS::traverseSetAndReturn (size_t set, ValidFunctor& vfunc, InvalidFunctor& ifunc) const
{
    std::vector<Position> vecPos;

    for (int j = 0; j != Way; ++j) {
        Position curPos (set, j);

        if (isPosValid (curPos)) {
            if (vfunc (entryKey (curPos), entryMapped (curPos))) {
                vecPos.push_back (curPos);
            }
        } else {
            if (ifunc (entryKey (curPos), entryMapped (curPos))) {
                vecPos.push_back (curPos);
            }
        }
    }

    return vecPos;
}

TEMPLATE_LIST
template <class PrintFunctor>
void
TEMPLATE_CLASS::print (std::ostream &os, PrintFunctor& pfunc) const
{
    for (int i = 0; i != Set; ++i) {
        os << "Set " << DEC << i << ":";

        for (int j = 0; j != Way; ++j) {
            Position curPos (i, j);

            if (isPosValid (curPos)) {
                pfunc (os, entryKey (curPos), entryMapped (curPos));
            } else {
                os << "(nil)";
            }

            os << " | ";
        }

        os << std::endl;
    }
}

#undef TEMPLATE_LIST
#undef TEMPLATE_CLASS

#endif // __BASE_TABLE_HH__
