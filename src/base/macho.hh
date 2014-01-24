/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#ifndef __BASE_MACHO_HH__
#define __BASE_MACHO_HH__

template <class State,
          class Event,
          class StateCompare = std::less<State>,
          class EventCompare = std::less<Event>>
class Macho
{
  public:
    typedef State state_type;
    typedef Event event_type;
    typedef StateCompare state_compare_type;
    typedef EventCompare event_compare_type;

  public:
    // Constructor.
    explicit Macho (state_type initState) : curState (initState) {}

  public:
    // Registers legal state transitions.
    // When the MACHO is in OLDSTATE and an EVENT occurs, the MACHO should turn
    // into NEWSTATE.
    int regStateEvent (state_type oldState, event_type event, state_type newState);

    // Registers the local default state.
    int regLocalDefaultState (state_type oldState, state_type newState);

    // Registers the global default state.
    int regGlobalDefaultState (state_type state);

    // Transfers the current state according to the given EVENT.
    state_type transfer (event_type event);

  private:
    // Types for default state transition table.
    typedef std::map<state_type, state_type> dstt_table_type;
    typedef typename dstt_table_type::key_type dstt_key_type;
    typedef typename dstt_table_type::mapped_type dstt_mapped_type;
    typedef typename dstt_table_type::value_type dstt_value_type;
    typedef typename dstt_table_type::iterator dstt_iterator;
    typedef typename dstt_table_type::const_iterator dstt_const_iterator;

    // Types for state transition table.
    typedef std::map<std::pair<state_type, event_type>, state_type> stt_table_type;
    typedef typename stt_table_type::key_type stt_key_type;
    typedef typename stt_table_type::mapped_type stt_mapped_type;
    typedef typename stt_table_type::value_type stt_value_type;
    typedef typename stt_table_type::iterator stt_iterator;
    typedef typename stt_table_type::const_iterator stt_const_iterator;

  private:
    // Constructor.
    // Must give the initial state.
    Macho (void) {}

  private:
    // Current state.
    // Must be initialized.
    state_type curState;

    // Global default state.
    state_type globalDefaultState;
    bool isGlobalDefaultStateInit;

    // Default state transition table.
    dstt_table_type dstt;

    // State transition table.
    stt_table_type stt;
};

#define TEMPLATE_LIST template<class State,        \
                               class Event,        \
                               class StateCompare, \
                               class EventCompare>
#define TEMPLATE_CLASS Macho<State,        \
                             Event,        \
                             StateCompare, \
                             EventCompare>

TEMPLATE_LIST
int
TEMPLATE_CLASS::regStateEvent (state_type oldState,
                               event_type event,
                               state_type newState)
{
    stt_key_type insertKey (oldState, event);
    stt_mapped_type insertMapped = newState;
    stt_value_type insertValue (insertKey, insertMapped);

    // Equavalence checking.
    stt_const_iterator cit = stt.find (insertKey);
    if (cit != stt.end ()) {
        if (cit->second != newState) {
            // Confliction occurs.
            return 0;
        } else {
            // Equavalent transition.
            return 1;
        }
    }

    // Inserts into the state transition table.
    stt.insert (insertValue);

    return 1;
}

TEMPLATE_LIST
int
TEMPLATE_CLASS::regLocalDefaultState (state_type oldState,
                                      state_type newState)
{
    dstt_key_type insertKey = oldState;
    dstt_mapped_type insertMapped = newState;
    dstt_value_type insertValue (insertKey, insertMapped);

    // Equavalence checking.
    dstt_const_iterator cit = dstt.find (insertKey);
    if (cit != dstt.end ()) {
        if (cit->second != insertMapped) {
            // Confliction occurs.
            return 0;
        } else {
            return 1;
        }
    }

    // Inserts into the default state transition table.
    dstt.insert (insertValue);

    return 1;
}

TEMPLATE_LIST
int
TEMPLATE_CLASS::regGlobalDefaultState (state_type state)
{
    // Confliction checking.
    if (isGlobalDefaultStateInit) {
        return (globalDefaultState == state) ? 1 : 0;
    }

    // Registers the global default state.
    isGlobalDefaultStateInit = true;
    globalDefaultState = state;
}

TEMPLATE_LIST
typename TEMPLATE_CLASS::state_type
TEMPLATE_CLASS::transfer (event_type event)
{
    state_type retState;

    stt_key_type sttKey (curState, event);
    stt_const_iterator i = stt.find (sttKey);

    if (i != stt.end ()) {
        retState = i->second;
    } else {
        dstt_key_type dsttKey = curState;
        dstt_const_iterator j = dstt.find (dsttKey);

        if (j != dstt.end ()) {
            retState = j->second;
        } else {
            retState = globalDefaultState;
        }
    }

    return retState;
}

#undef TEMPLATE_LIST
#undef TEMPLATE_CLASS
#endif // __BASE_MACHO_HH__
