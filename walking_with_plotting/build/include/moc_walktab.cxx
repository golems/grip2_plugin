/****************************************************************************
** Meta object code from reading C++ file 'walktab.h'
**
** Created: Tue Apr 8 11:04:41 2014
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
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       9,    8,    8,    8, 0x08,
      27,    8,    8,    8, 0x08,
      44,    8,    8,    8, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_WalkTab[] = {
    "WalkTab\0\0setStartPressed()\0setGoalPressed()\0"
    "update()\0"
};

void WalkTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        WalkTab *_t = static_cast<WalkTab *>(_o);
        switch (_id) {
        case 0: _t->setStartPressed(); break;
        case 1: _t->setGoalPressed(); break;
        case 2: _t->update(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
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
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
