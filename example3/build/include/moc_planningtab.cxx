/****************************************************************************
** Meta object code from reading C++ file 'planningtab.h'
**
** Created: Wed Apr 2 14:30:25 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/planningtab.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'planningtab.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PlanningTab[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      13,   12,   12,   12, 0x08,
      31,   12,   12,   12, 0x08,
      48,   12,   12,   12, 0x08,
      72,   12,   12,   12, 0x08,
      95,   12,   12,   12, 0x08,
     120,   12,   12,   12, 0x08,
     139,   12,   12,   12, 0x08,
     157,   12,   12,   12, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PlanningTab[] = {
    "PlanningTab\0\0setStartPressed()\0"
    "setGoalPressed()\0setPredefStartPressed()\0"
    "setPredefGoalPressed()\0relocateObjectsPressed()\0"
    "showStartPressed()\0showGoalPressed()\0"
    "doPlanPressed()\0"
};

void PlanningTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PlanningTab *_t = static_cast<PlanningTab *>(_o);
        switch (_id) {
        case 0: _t->setStartPressed(); break;
        case 1: _t->setGoalPressed(); break;
        case 2: _t->setPredefStartPressed(); break;
        case 3: _t->setPredefGoalPressed(); break;
        case 4: _t->relocateObjectsPressed(); break;
        case 5: _t->showStartPressed(); break;
        case 6: _t->showGoalPressed(); break;
        case 7: _t->doPlanPressed(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData PlanningTab::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PlanningTab::staticMetaObject = {
    { &GripTab::staticMetaObject, qt_meta_stringdata_PlanningTab,
      qt_meta_data_PlanningTab, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PlanningTab::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PlanningTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PlanningTab::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PlanningTab))
        return static_cast<void*>(const_cast< PlanningTab*>(this));
    if (!strcmp(_clname, "com.gatech.Grip2.GripTab/1.0"))
        return static_cast< GripTab*>(const_cast< PlanningTab*>(this));
    return GripTab::qt_metacast(_clname);
}

int PlanningTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GripTab::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
