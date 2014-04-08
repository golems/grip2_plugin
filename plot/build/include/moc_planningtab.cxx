/****************************************************************************
** Meta object code from reading C++ file 'planningtab.h'
**
** Created: Mon Apr 7 22:04:34 2014
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
static const uint qt_meta_data_PlotTab[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       9,    8,    8,    8, 0x09,
      22,   18,    8,    8, 0x09,
      49,    8,    8,    8, 0x09,
      66,    8,    8,    8, 0x09,
      84,    8,    8,    8, 0x09,
     106,    8,    8,    8, 0x09,
     125,    8,    8,    8, 0x09,
     138,    8,    8,    8, 0x09,
     157,    8,    8,    8, 0x09,
     170,    8,    8,    8, 0x09,
     183,    8,    8,    8, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_PlotTab[] = {
    "PlotTab\0\0update()\0pos\0contextMenuRequest(QPoint)\0"
    "addRandomGraph()\0removeAllGraphs()\0"
    "removeSelectedGraph()\0selectDartStream()\0"
    "drawPlugin()\0selectionChanged()\0"
    "moveLegend()\0mouseWheel()\0mousePress()\0"
};

void PlotTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PlotTab *_t = static_cast<PlotTab *>(_o);
        switch (_id) {
        case 0: _t->update(); break;
        case 1: _t->contextMenuRequest((*reinterpret_cast< QPoint(*)>(_a[1]))); break;
        case 2: _t->addRandomGraph(); break;
        case 3: _t->removeAllGraphs(); break;
        case 4: _t->removeSelectedGraph(); break;
        case 5: _t->selectDartStream(); break;
        case 6: _t->drawPlugin(); break;
        case 7: _t->selectionChanged(); break;
        case 8: _t->moveLegend(); break;
        case 9: _t->mouseWheel(); break;
        case 10: _t->mousePress(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PlotTab::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PlotTab::staticMetaObject = {
    { &GripTab::staticMetaObject, qt_meta_stringdata_PlotTab,
      qt_meta_data_PlotTab, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PlotTab::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PlotTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PlotTab::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PlotTab))
        return static_cast<void*>(const_cast< PlotTab*>(this));
    if (!strcmp(_clname, "com.gatech.Grip2.GripTab/1.0"))
        return static_cast< GripTab*>(const_cast< PlotTab*>(this));
    return GripTab::qt_metacast(_clname);
}

int PlotTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GripTab::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
