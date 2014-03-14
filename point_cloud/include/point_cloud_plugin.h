#ifndef POINT_CLOUD_PLUGIN_H
#define POINT_CLOUD_PLUGIN_H

#include "ui_point_cloud_plugin.h"
#include "PCDLoader.h"
#include "qtWidgets/include/GripTab.h"
#include "qtWidgets/include/TreeViewReturn.h"
#include "osgNodes/include/ViewerWidget.h"

class PointCloudPlugin : public GripTab
{
    Q_OBJECT
    Q_INTERFACES(GripTab)
public:
    PointCloudPlugin(QWidget *parent = 0);
    ~PointCloudPlugin();

private:
    Ui::PointCloudPlugin *ui;
    PCDLoader* loader;

public slots:
    void testslot();

public:
    void Load(TreeViewReturn* ret, ViewerWidget* viewer);
    /* Fire in relation to dynamic simulation */
    /* Suitable for controllers */
    void GRIPEventSimulationBeforeTimestep();
    void GRIPEventSimulationAfterTimestep();
    void GRIPEventSimulationStart();
    void GRIPEventSimulationStop();

    void GRIPEventPlaybackBeforeFrame();

    /**
     * \brief called from the main window whenever the simulation history slider is being played
     * This method is executed after every playback time step
     */
    void GRIPEventPlaybackAfterFrame();

    /**
     * \brief called from the main window whenever the simulation history slider is being played
     * This method is executed at the start of the playback
     */
    void GRIPEventPlaybackStart();

    /**
     * \brief called from the main window whenever the simulation history slider is being played
     * This method is executed at the end of the playback
     */
    void GRIPEventPlaybackStop();

    /**
     * \brief called from the main window when a new object is selected in the treeview
     */
    void GRIPEventTreeViewSelectionChanged();
    void Refresh();

};

#endif // TESTGRIPPLUGIN_H
