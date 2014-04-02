/****************************************************************************
** Meta object code from reading C++ file 'walktab.h'
**
** Created: Wed Apr 2 16:16:55 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/walktab.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'walktab.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_WalkTab[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       9,    8,    8,    8, 0x08,
      27,    8,    8,    8, 0x08,
      69,   44,    8,    8, 0x08,
     127,  112,    8,    8, 0x28,
     173,  165,    8,    8, 0x28,
     216,  204,    8,    8, 0x08,
     244,    8,    8,    8, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_WalkTab[] = {
    "WalkTab\0\0setStartPressed()\0setGoalPressed()\0"
    "dx,left,period,sameFrame\0"
    "moveFoot(Eigen::VectorXd,bool,size_t,bool)\0"
    "dx,left,period\0moveFoot(Eigen::VectorXd,bool,size_t)\0"
    "dx,left\0moveFoot(Eigen::VectorXd,bool)\0"
    "desiredDofs\0setTorques(Eigen::VectorXd)\0"
    "Refresh()\0"
};

void WalkTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        WalkTab *_t = static_cast<WalkTab *>(_o);
        switch (_id) {
        case 0: _t->setStartPressed(); break;
        case 1: _t->setGoalPressed(); break;
        case 2: _t->moveFoot((*reinterpret_cast< const Eigen::VectorXd(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< size_t(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4]))); break;
        case 3: _t->moveFoot((*reinterpret_cast< const Eigen::VectorXd(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< size_t(*)>(_a[3]))); break;
        case 4: _t->moveFoot((*reinterpret_cast< const Eigen::VectorXd(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 5: _t->setTorques((*reinterpret_cast< const Eigen::VectorXd(*)>(_a[1]))); break;
        case 6: _t->Refresh(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData WalkTab::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject WalkTab::staticMetaObject = {
    { &GripTab::staticMetaObject, qt_meta_stringdata_WalkTab,
      qt_meta_data_WalkTab, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &WalkTab::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *WalkTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *WalkTab::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_WalkTab))
        return static_cast<void*>(const_cast< WalkTab*>(this));
    if (!strcmp(_clname, "com.gatech.Grip2.GripTab/1.0"))
        return static_cast< GripTab*>(const_cast< WalkTab*>(this));
    return GripTab::qt_metacast(_clname);
}

int WalkTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GripTab::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
