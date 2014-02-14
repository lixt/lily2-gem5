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
    typedef void (*Callback) (void);
    typedef State state_type;
    typedef Event event_type;
    typedef Callback callback_type;
    typedef StateCompare state_compare_type;
    typedef EventCompare event_compare_type;
    typedef std::less<callback_type> callback_compare_type;

  private:
    static state_compare_type state_less;
    static event_compare_type event_less;
    static callback_compare_type callback_less;

  public:
    // Registers legal state transitions.
    // When the MACHO is in OLDSTATE and an EVENT occurs, the MACHO should turn
    // into NEWSTATE.
    int regStateEvent (const state_type& oldState,
                       const event_type& event,
                       const state_type& newState,
                       const callback_type& callback = NULL);

    // Registers the local default state.
    int regLocalDefaultState (const state_type& oldState,
                              const state_type& newState,
                              const callback_type& callback = NULL);

    // Registers the global default state.
    int regGlobalDefaultState (const state_type& state,
                               const callback_type& callback = NULL);

    // Transfers the current state according to the given EVENT.
    state_type transfer (const event_type& event);

  private:
    // Pair of state and event.
    typedef struct state_event {
        state_type state;
        event_type event;

        state_event (const state_type& state, const event_type& event) :
            state (state), event (event) {}

        friend bool operator< (const state_event& x, const state_event& y)
        {
            if (state_less (x.state, y.state)) {
                return true;
            }

            if (state_less (y.state, x.state)) {
                return false;
            }

            return event_less (x.event, y.event);
        }
    } state_event_type;

    // Pair of state and callback.
    typedef struct state_callback {
        state_type state;
        callback_type callback;

        state_callback (const state_type& state, const callback_type& callback) :
            state (state), callback (callback) {}

        friend bool operator< (const state_callback& x, const state_callback& y)
        {
            if (state_less (x.state, y.state)) {
                return true;
            }

            if (state_less (y.state, x.state)) {
                return false;
            }

            return callback_less (x.callback, y.callback);
        }

        friend bool operator!= (const state_callback& x, const state_callback& y)
        {
            return (x < y) && (y < x);
        }
    } state_callback_type;

    // Types for global default state transition table.
    typedef std::map<state_callback_type, int>        gdstt_table_type;
    typedef typename gdstt_table_type::key_type       gdstt_key_type;
    typedef typename gdstt_table_type::mapped_type    gdstt_mapped_type;
    typedef typename gdstt_table_type::value_type     gdstt_value_type;
    typedef typename gdstt_table_type::iterator       gdstt_iterator;
    typedef typename gdstt_table_type::const_iterator gdstt_const_iterator;


    // Types for default state transition table.
    typedef std::map<state_type, state_callback_type> ldstt_table_type;
    typedef typename ldstt_table_type::key_type       ldstt_key_type;
    typedef typename ldstt_table_type::mapped_type    ldstt_mapped_type;
    typedef typename ldstt_table_type::value_type     ldstt_value_type;
    typedef typename ldstt_table_type::iterator       ldstt_iterator;
    typedef typename ldstt_table_type::const_iterator ldstt_const_iterator;

    // Types for state transition table.
    typedef std::map<state_event_type, state_callback_type> stt_table_type;
    typedef typename stt_table_type::key_type               stt_key_type;
    typedef typename stt_table_type::mapped_type            stt_mapped_type;
    typedef typename stt_table_type::value_type             stt_value_type;
    typedef typename stt_table_type::iterator               stt_iterator;
    typedef typename stt_table_type::const_iterator         stt_const_iterator;

  public:
    // Constructor.
    explicit Macho (const state_type& initState)
    {
        this->curState = initState;
    }

    // Accessor and mutator of the current state.
    const state_type& getCurState (void) const
    {
        return curState;
    }
    void setCurState (const state_type& curState)
    {
        this->curState = curState;
    }

  private:
    // Default constructor is illegal.
    Macho (void) {}

  private:
    // Current state.
    state_type curState;

    // Global default state transition table.
    gdstt_table_type gdstt;

    // Local default state transition table.
    ldstt_table_type ldstt;

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
TEMPLATE_CLASS::regStateEvent (const state_type& oldState,
                               const event_type& event,
                               const state_type& newState,
                               const callback_type& callback)
{
    stt_key_type insertKey (oldState, event);
    stt_mapped_type insertMapped (newState, callback);
    stt_value_type insertValue (insertKey, insertMapped);
    stt_const_iterator cit = stt.find (insertKey);

    if (cit != stt.end ()) {
        if (cit->second != insertMapped) {
            // Same key, different mapped.
            return 0;
        } else {
            // Same key, same mapped.
            return 1;
        }
    } else {
        stt.insert (insertValue);
        return 1;
    }
}

TEMPLATE_LIST
int
TEMPLATE_CLASS::regLocalDefaultState (const state_type& oldState,
                                      const state_type& newState,
                                      const callback_type& callback)
{
    ldstt_key_type insertKey = oldState;
    ldstt_mapped_type insertMapped (newState, callback);
    ldstt_value_type insertValue (insertKey, insertMapped);
    ldstt_const_iterator cit = ldstt.find (insertKey);

    if (cit != ldstt.end ()) {
        if (cit->second != insertMapped) {
            // Same key, different mapped.
            return 0;
        } else {
            // Same key, same mapped.
            return 1;
        }
    } else {
        ldstt.insert (insertValue);
        return 1;
    }
}

TEMPLATE_LIST
int
TEMPLATE_CLASS::regGlobalDefaultState (const state_type& state,
                                       const callback_type& callback)
{
    // Conflict checking.
    state_callback_type insert (state, callback);
    gdstt_const_iterator cit = gdstt.find (insert);
    if (gdstt.size () != 0) {
        if (cit != gdstt.end ()) {
            return 0;
        } else {
            return 1;
        }
    } else {
        gdstt.insert (insert, 0); // 0 is meaningless.
        return 1;
    }
}

TEMPLATE_LIST
typename TEMPLATE_CLASS::state_type
TEMPLATE_CLASS::transfer (const event_type& event)
{
    state_type retState;
    callback_type callback;

    stt_key_type sttKey (curState, event);
    stt_const_iterator i = stt.find (sttKey);

    if (i != stt.end ()) {
        retState = (i->second).state;
        callback = (i->second).callback;
    } else {
        ldstt_key_type ldsttKey = curState;
        ldstt_const_iterator j = ldstt.find (ldsttKey);

        if (j != ldstt.end ()) {
            retState = (j->second).state;
            callback = (j->second).callback;
        } else {
            gdstt_const_iterator k = gdstt.begin ();

            if (k == gdstt.end ()) {
                assert (0);
            } else {
                retState = (k->first).state;
                callback = (k->first).callback;
            }
        }
    }

    // Calls the callbacks.
    callback ();

    return retState;
}

#undef TEMPLATE_LIST
#undef TEMPLATE_CLASS
#endif // __BASE_MACHO_HH__
