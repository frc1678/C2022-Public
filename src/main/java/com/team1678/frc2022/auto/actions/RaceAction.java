package com.team1678.frc2022.auto.actions;

/**
 * Composite action, running all sub-actions at the same time All actions are
 * started then updated until all actions
 * report being done.
 *
 */
public class RaceAction implements Action {

    private final Action[] mActions;

    private final Action leadAction;

    public RaceAction(Action leadAction, Action... actions) {
        mActions = actions;
        this.leadAction = leadAction;
    }

    @Override
    public boolean isFinished() {
        if (leadAction.isFinished()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        leadAction.update();
        for (Action action : mActions) {
            action.update();
        }
    }

    @Override
    public void done() {
        leadAction.done();
        for (Action action : mActions) {
            action.done();
        }
    }

    @Override
    public void start() {
        leadAction.start();
        for (Action action : mActions) {
            action.start();
        }
    }
}
