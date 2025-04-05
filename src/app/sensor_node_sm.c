// State machine includes
#include <zephyr/smf.h>
#include <dk_buttons_and_leds.h>

#include "ble_interface.h"
#include "sensor_node_sm.h"

// Forward declaration of state table
static const struct smf_state endnode_states[];

/* User defined object */
struct s_object {
        /* This must be first */
        struct smf_ctx ctx;

        /* Other state specific data add here */
} s_obj;

// State NotSynced
static void notsynced_entry(void *o)
{
    // Start advertising
    ble_start_advertising();

    // Turn off LED1 to indicate not synced
    dk_set_led_off(DK_LED1);

    printk("Started Advertising... Waiting for periodic sync info..\n");    
}

// State Synced
static void synced_entry(void *o)
{
    // Turn on LED1 to indicate synced
    dk_set_led_on(DK_LED1);
    
    printk("Synced to collector\n");
}

static void synced_exit(void *o)
{
    // Delete sync
    ble_delete_per_adv_sync();
    
    // Turn off LED1 when exiting synced state
    dk_set_led_off(DK_LED1);
}

/* Populate state table */
static const struct smf_state endnode_states[] = {
		// NotSynced State
        [NotSynced] = {
            .entry = notsynced_entry,
            .run = NULL,
            .exit = NULL
        },
		// Synced State
        [Synced] = {
            .entry = synced_entry,
            .run = NULL,
            .exit = synced_exit
        }
};

// State machine initialization
void sensor_node_sm_init(void)
{
    // State machine initialization
	smf_set_initial(SMF_CTX(&s_obj), &endnode_states[NotSynced]);
}

void sensor_node_sm_set_state(enum endnode_state state)
{
    smf_set_state(SMF_CTX(&s_obj), &endnode_states[state]);
}

// State machine processing
int sensor_node_sm_run(void)
{
    return smf_run_state(SMF_CTX(&s_obj));
}