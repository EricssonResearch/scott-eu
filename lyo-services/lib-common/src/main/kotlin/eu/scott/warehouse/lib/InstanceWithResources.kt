package eu.scott.warehouse.lib

import org.eclipse.lyo.oslc4j.core.model.IExtendedResource

interface ResourceArray {
    fun asResourceArray(): Array<Object>
}


data class InstanceWithResources<T : IExtendedResource>(val instance: T,
                                                        val resources: Collection<IExtendedResource>) :
    ResourceArray {
    override fun asResourceArray(): Array<Object> {
        val resources: MutableList<Object> = ArrayList()
        resources.add(instance as Object)
        resources.addAll(resources)
        return resources.toTypedArray()
    }
}

data class InstanceMultiWithResources<T : IExtendedResource>(val instance: Collection<T>,
                                                             val resources: Collection<IExtendedResource>) :
    ResourceArray {
    override fun asResourceArray(): Array<Object> {
        val resources: MutableList<Object> = ArrayList()
        resources.addAll(instance as Collection<Object>)
        resources.addAll(resources)
        return resources.toTypedArray()
    }
}
